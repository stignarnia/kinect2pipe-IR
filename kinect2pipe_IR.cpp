#include <iostream>
#include <libfreenect2/logger.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <linux/videodev2.h>
#include <sys/inotify.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <thread>
#include <cstring>
#include "kinect2pipe_IR.h"

extern "C" {
    #include <libswscale/swscale.h>
    #include <libavutil/pixfmt.h>
}

using namespace std;
using namespace libfreenect2;

static kinect2pipe_IR* g_instance = nullptr;

static void signalHandler(int) {
    if (g_instance) g_instance->shutdown();
}

kinect2pipe_IR::kinect2pipe_IR() {
    g_instance = this;

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
    this->started    = false;
    this->shouldStop = false;

    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    libfreenect2::setGlobalLogger(nullptr);
}

void kinect2pipe_IR::shutdown() {
    cout << "shutting down" << endl;
    lock_guard<mutex> lk(this->cvMutex);
    this->shouldStop = true;
    this->cv.notify_one();
}

void kinect2pipe_IR::openLoopback(const char* loopbackDev) {
    if (!this->openV4L2LoopbackDevice(loopbackDev, OUTPUT_WIDTH, OUTPUT_HEIGHT)) {
        exit(1);
    }
    this->writeBlankFrame();
    if (!this->openInotifyWatcher(loopbackDev)) {
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

bool kinect2pipe_IR::openInotifyWatcher(const char* loopbackDev) {
    this->watcherThread = thread(&kinect2pipe_IR::inotifyWatcher, this, loopbackDev);
    this->watcherThread.detach();
    return true;
}

void kinect2pipe_IR::inotifyWatcher(const char* loopbackDev) {
    int ifd = inotify_init();
    if (ifd < 0) {
        perror("inotify_init");
        exit(-1);
    }

    if (inotify_add_watch(ifd, loopbackDev, IN_OPEN | IN_CLOSE_WRITE | IN_CLOSE_NOWRITE) < 0) {
        perror("inotify_add_watch");
        exit(-1);
    }

    int openCount = 0;

    char buf[sizeof(struct inotify_event) + NAME_MAX + 1];

    while (true) {
        ssize_t len = read(ifd, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EINTR) continue;
            perror("inotify read");
            break;
        }

        const struct inotify_event* ev = reinterpret_cast<const struct inotify_event*>(buf);

        if (ev->mask & IN_OPEN) {
            openCount++;
            cout << "consumer opened device (count=" << openCount << ")" << endl;
            if (openCount == 1) {
                lock_guard<mutex> lk(this->cvMutex);
                this->started = true;
                this->cv.notify_one();
            }
        } else if (ev->mask & (IN_CLOSE_WRITE | IN_CLOSE_NOWRITE)) {
            openCount--;
            cout << "consumer closed device (count=" << openCount << ")" << endl;
            if (openCount <= 0) {
                cout << "last consumer gone, stopping" << endl;
                this->shutdown();
                close(ifd);
                return;
            }
        }
    }

    close(ifd);
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

    while (true) {
        {
            unique_lock<mutex> lk(this->cvMutex);
            if (this->shouldStop) break;
        }

        if (!listener.waitForNewFrame(frames, 1000)) {
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

void kinect2pipe_IR::run() {
    // Wait until a consumer opens the device, or until shutdown() is called
    {
        unique_lock<mutex> lk(this->cvMutex);
        this->cv.wait(lk, [this]{ return this->started || this->shouldStop; });
        if (this->shouldStop) {
            exit(0);
        }
    }

    cout << "starting kinect2 IR capture" << endl;
    if (!this->openKinect2Device()) {
        exit(-1);
    }
    exit(0);
}

void kinect2pipe_IR::writeBlankFrame() {
    memset(this->imageBuffer,                    0x10, YUV_BUFFER_Y_LEN);
    memset(this->imageBuffer + YUV_BUFFER_Y_LEN, 0x80, YUV_BUFFER_UV_LEN * 2);
    write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN);
}