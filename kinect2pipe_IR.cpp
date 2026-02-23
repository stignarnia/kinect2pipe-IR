#include <iostream>
#include <libfreenect2/logger.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <dirent.h>
#include <algorithm>
#include <cstring>
#include "kinect2pipe_IR.h"

extern "C" {
    #include <libswscale/swscale.h>
    #include <libavutil/pixfmt.h>
}

using namespace std;
using namespace libfreenect2;

kinect2pipe_IR::kinect2pipe_IR() {
    this->normBuf = (float*)malloc(KINECT2_IMAGE_WIDTH * KINECT2_IMAGE_HEIGHT * sizeof(float));

    this->sws = sws_getContext(
        KINECT2_IMAGE_WIDTH, KINECT2_IMAGE_HEIGHT, AV_PIX_FMT_GRAYF32,
        OUTPUT_WIDTH,        OUTPUT_HEIGHT,        AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, nullptr, nullptr, nullptr);

    memset(this->srcPtr,    0, sizeof(this->srcPtr));
    memset(this->srcStride, 0, sizeof(this->srcStride));

    this->srcPtr[0]    = reinterpret_cast<uint8_t*>(this->normBuf);
    this->srcStride[0] = KINECT2_IMAGE_WIDTH * sizeof(float);

    this->imageBuffer = (uint8_t*)calloc(1, YUV_BUFFER_LEN);
    this->dstPtr[0] = this->imageBuffer;
    this->dstPtr[1] = this->imageBuffer + YUV_BUFFER_Y_LEN;
    this->dstPtr[2] = this->imageBuffer + YUV_BUFFER_Y_LEN + YUV_BUFFER_UV_LEN;
    this->dstPtr[3] = nullptr;

    this->dstStride[0] = OUTPUT_WIDTH;
    this->dstStride[1] = OUTPUT_WIDTH / 2;
    this->dstStride[2] = OUTPUT_WIDTH / 2;
    this->dstStride[3] = 0;

    this->v4l2Device = 0;
    this->shouldStop = false;

    libfreenect2::setGlobalLogger(nullptr);
}

void kinect2pipe_IR::openLoopback(const char* loopbackDev) {
    if (!this->openV4L2LoopbackDevice(loopbackDev, OUTPUT_WIDTH, OUTPUT_HEIGHT)) {
        exit(1);
    }
    this->writeBlankFrame();
    if (!this->openProcMonitorV4L2LoopbackDevice(loopbackDev)) {
        exit(1);
    }
}

bool kinect2pipe_IR::openV4L2LoopbackDevice(const char* loopbackDev, int width, int height) {
    this->v4l2Device = open(loopbackDev, O_WRONLY);
    if (this->v4l2Device < 0) {
        cerr << "failed to open v4l2loopback device: " << errno << endl;
        return false;
    }

    struct v4l2_format fmt{};
    fmt.type                 = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width        = width;
    fmt.fmt.pix.height       = height;
    fmt.fmt.pix.pixelformat  = V4L2_PIX_FMT_YUV420;
    fmt.fmt.pix.sizeimage    = YUV_BUFFER_LEN;
    fmt.fmt.pix.field        = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = width;
    fmt.fmt.pix.colorspace   = V4L2_COLORSPACE_SRGB;

    if (ioctl(this->v4l2Device, VIDIOC_S_FMT, &fmt) < 0) {
        cerr << "failed to issue ioctl to v4l2loopback device: " << errno << endl;
        return false;
    }
    return true;
}

bool kinect2pipe_IR::openKinect2Device() {
    SyncMultiFrameListener listener(Frame::Ir);
    Freenect2Device* dev;

    if (freenect2.enumerateDevices() == 0) {
        cerr << "unable to find a kinect2 device to connect to" << endl;
        return false;
    }

    dev = freenect2.openDefaultDevice();
    if (!dev) {
        cerr << "failed to open kinect2 device" << endl;
        return false;
    }

    dev->setIrAndDepthFrameListener(&listener);

    if (!dev->startStreams(false, true)) {
        cerr << "unable to start kinect2 ir stream" << endl;
        dev->close();
        return false;
    }

    cout << "kinect2 IR stream started" << endl;

    // Stream frames until the watcher tells us to stop
    while (true) {
        {
            unique_lock<mutex> lk(this->stopMutex);
            if (this->shouldStop) break;
        }

        if (!listener.waitForNewFrame(frames, 1000)) {
            // waitForNewFrame timed out — check shouldStop and retry
            continue;
        }

        if (!this->handleFrame(frames[Frame::Ir])) {
            listener.release(frames);
            break;
        }

        listener.release(frames);
    }

    cout << "stopping kinect2 IR stream" << endl;
    this->writeBlankFrame();
    dev->stop();
    dev->close();
    return true;
}

bool kinect2pipe_IR::handleFrame(Frame* frame) {
    const float* src = reinterpret_cast<const float*>(frame->data);
    const int    n   = KINECT2_IMAGE_WIDTH * KINECT2_IMAGE_HEIGHT;
    for (int i = 0; i < n; ++i) {
        this->normBuf[i] = src[i] / IR_MAX_VALUE;
    }

    sws_scale(this->sws,
              this->srcPtr,  this->srcStride, 0, KINECT2_IMAGE_HEIGHT,
              this->dstPtr,  this->dstStride);

    return write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN) > 0;
}

bool kinect2pipe_IR::openProcMonitorV4L2LoopbackDevice(const char* loopbackDev) {
    this->watcherThread = thread(&kinect2pipe_IR::procMonitorV4L2LoopbackDevice, this, loopbackDev);
    this->watcherThread.detach();
    return true;
}

void kinect2pipe_IR::procMonitorV4L2LoopbackDevice(const char* loopbackDev) {
    const string current_pid = to_string(getpid());
    const string target      = loopbackDev;

    // Track whether we have ever seen a consumer.
    // We don't start/stop the Kinect here — run() starts it immediately.
    // Our only job: once we've seen at least one consumer come and then fully go,
    // signal the stream loop to stop cleanly, then exit.
    bool everSawConsumer = false;

    while (true) {
        this_thread::sleep_for(chrono::seconds(1));

        int consumers = 0;

        DIR* proc_dir = opendir("/proc");
        if (!proc_dir) {
            perror("opendir /proc");
            exit(-1);
        }

        struct dirent* entry;
        while ((entry = readdir(proc_dir)) != nullptr) {
            string pid_str(entry->d_name);
            if (!all_of(pid_str.begin(), pid_str.end(), ::isdigit)) continue;
            if (pid_str == current_pid) continue;

            string fd_dir_path = "/proc/" + pid_str + "/fd";
            DIR*   fd_dir      = opendir(fd_dir_path.c_str());
            if (!fd_dir) continue;

            struct dirent* fd_entry;
            while ((fd_entry = readdir(fd_dir)) != nullptr) {
                if (strcmp(fd_entry->d_name, ".") == 0 ||
                    strcmp(fd_entry->d_name, "..") == 0)
                    continue;

                string  fd_path = fd_dir_path + "/" + fd_entry->d_name;
                char    link_target[PATH_MAX];
                ssize_t len = readlink(fd_path.c_str(), link_target, sizeof(link_target) - 1);
                if (len == -1) continue;
                link_target[len] = '\0';

                if (target == link_target) {
                    consumers++;
                }
            }
            closedir(fd_dir);
        }
        closedir(proc_dir);

        if (consumers > 0) {
            everSawConsumer = true;
        } else if (everSawConsumer) {
            // Had a consumer, now gone — signal the stream loop to stop cleanly
            cout << "last consumer gone, stopping" << endl;
            {
                lock_guard<mutex> lk(this->stopMutex);
                this->shouldStop = true;
            }
            this->stopCondition.notify_one();
            // run() will exit() after dev->stop()/close(), so we just return
            return;
        }
    }
}

void kinect2pipe_IR::run() {
    // Start streaming immediately — don't wait for a consumer.
    // The Kinect must be hot and writing frames BEFORE howdy opens the device,
    // otherwise cv2.VideoCapture.grab() fails and howdy never registers as a consumer.
    cout << "starting kinect2 IR capture" << endl;
    if (!this->openKinect2Device()) {
        exit(-1);
    }
    // openKinect2Device returned cleanly (shouldStop was set by watcher).
    // Let systemd restart us.
    exit(0);
}

void kinect2pipe_IR::writeBlankFrame() {
    memset(this->imageBuffer,                    0x10, YUV_BUFFER_Y_LEN);
    memset(this->imageBuffer + YUV_BUFFER_Y_LEN, 0x80, YUV_BUFFER_UV_LEN * 2);
    write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN);
}