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
#include "kinect2pipe.h"
extern "C" {
    #include <libswscale/swscale.h>
}
using namespace std;
using namespace libfreenect2;

Kinect2Pipe::Kinect2Pipe () {
    this->sws = sws_getContext(KINECT2_IMAGE_WIDTH, KINECT2_IMAGE_HEIGHT, AV_PIX_FMT_RGB32, OUTPUT_WIDTH, OUTPUT_HEIGHT, AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);
    memset(this->srcPtr, 0, sizeof(uint8_t*) * 4);
    this->srcStride[0] = KINECT2_IMAGE_WIDTH*4;
    this->srcStride[1] = 0;
    this->srcStride[2] = 0;
    this->srcStride[3] = 0;

    this->imageBuffer = (uint8_t*)calloc(1, YUV_BUFFER_LEN);
    this->dstPtr[0] = this->imageBuffer;
    this->dstPtr[1] = this->imageBuffer + YUV_BUFFER_Y_LEN;
    this->dstPtr[2] = this->imageBuffer + YUV_BUFFER_Y_LEN + YUV_BUFFER_UV_LEN;
    this->dstPtr[3] = nullptr;
    this->dstStride[0] = OUTPUT_WIDTH;
    this->dstStride[1] = OUTPUT_WIDTH/2;
    this->dstStride[2] = OUTPUT_WIDTH/2;
    this->dstStride[3] = 0;

    this->v4l2Device = 0;
    this->openCount = 0;
    this->started = false;

    libfreenect2::setGlobalLogger(nullptr);
}

void Kinect2Pipe::openLoopback(const char *loopbackDev) {
    if (!this->openV4L2LoopbackDevice(loopbackDev, OUTPUT_WIDTH, OUTPUT_HEIGHT)) {
        exit(1);
    }
    this->writeBlankFrame();
    if (!this->openProcMonitorV4L2LoopbackDevice(loopbackDev)) {
        exit(1);
    }
}

bool Kinect2Pipe::openV4L2LoopbackDevice(const char* loopbackDev, int width, int height) {
    this->v4l2Device = open(loopbackDev, O_WRONLY);
    if (this->v4l2Device < 0) {
        cerr << "failed to open v4l2loopback device: " << errno << endl;
        return false;
    }
    struct v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    fmt.fmt.pix.sizeimage = fmt.fmt.pix.width * fmt.fmt.pix.height * 1.5;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = fmt.fmt.pix.width;
    fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

    if (ioctl(this->v4l2Device, VIDIOC_S_FMT, &fmt) < 0) {
        cerr << "failed to issue ioctl to v4l2loopback device: " << errno << endl;
        return false;
    }

    return true;
}

bool Kinect2Pipe::openKinect2Device() {
    SyncMultiFrameListener listener(Frame::Color);
    Freenect2Device *dev;
    if (freenect2.enumerateDevices() == 0) {
        cerr << "unable to find a kinect2 device to connect to" << endl;
        return false;
    }
    dev = freenect2.openDefaultDevice();
    dev->setColorFrameListener(&listener);
    if (!dev->startStreams(true, false)) {
        cerr << "unable to start kinect2 rgb stream" << endl;
        return false;
    }
    while (this->started) {
        if (!listener.waitForNewFrame(frames, 2000)) {
            cerr << "timeout waiting for frame" << endl;
            return false;
        }
        if (!this->handleFrame(frames[Frame::Color])) {
            return false;
        }
        listener.release(frames);
    }
    this->writeBlankFrame();
    dev->stop();
    dev->close();
    return true;
}

bool Kinect2Pipe::handleFrame(Frame* frame) {
    this->srcPtr[0] = frame->data;
    sws_scale(this->sws, this->srcPtr, this->srcStride, 0, KINECT2_IMAGE_HEIGHT, this->dstPtr, this->dstStride);
    return write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN) > 0;
}

bool Kinect2Pipe::openProcMonitorV4L2LoopbackDevice(const char* loopbackDev) {
    this->watcherThread = thread (&Kinect2Pipe::procMonitorV4L2LoopbackDevice, this, loopbackDev);
    this->watcherThread.detach();
    return true;
}
void Kinect2Pipe::procMonitorV4L2LoopbackDevice(const char* loopbackDev) {
    // We'll own the lock on this until we're ready to rumble.
    unique_lock<mutex> lk(this->startMutex);
    this->startCondition.wait(lk, [this]{ return !this->started; });

    const string current_pid = to_string(getpid());
    const string target = loopbackDev;

    while (true) {
		// Open /proc directory
		DIR *proc_dir = opendir("/proc");
		if (!proc_dir) {
			perror("opendir /proc");
			exit(-1);
		}

		// Iterate over entries in /proc
		struct dirent *entry;
		while ((entry = readdir(proc_dir)) != nullptr) {
			string pid_str(entry->d_name);
			if (!all_of(pid_str.begin(), pid_str.end(), ::isdigit)) continue; // skip non-numeric names
			if (pid_str == current_pid) continue; // skip current process

			// Build path to fd directory
			string fd_dir_path = "/proc/" + pid_str + "/fd";
			DIR *fd_dir = opendir(fd_dir_path.c_str());
			if (!fd_dir) continue; // can't open fd dir; maybe insufficient permission

			struct dirent *fd_entry;
			while ((fd_entry = readdir(fd_dir)) != nullptr) {
				// skip . and ..
				if (strcmp(fd_entry->d_name, ".") == 0 || strcmp(fd_entry->d_name, "..") == 0)
					continue;

				// path to fd symlink
				string fd_path = fd_dir_path + "/" + fd_entry->d_name;

				// readlink to get target
				char link_target[PATH_MAX];
				ssize_t len = readlink(fd_path.c_str(), link_target, sizeof(link_target)-1);
				if (len == -1) continue; // not a symlink or error
				link_target[len] = '\0';

				if (target == link_target) {
					this->openCount++;
					if (!this->started) {
						this->started = true;
						lk.unlock();
						this->startCondition.notify_one();
					}
				}
			}
			closedir(fd_dir);
		}
		closedir(proc_dir);

		if(this->openCount == 0 && this->started) {
			cout << "closing device since no one is watching" << endl;
			this->started = false;
			lk.lock();
		}

		this->openCount = 0;
		this_thread::sleep_for(chrono::seconds(1));
    }
}

void Kinect2Pipe::run() {
    while(true) {
        unique_lock<mutex> lk(this->startMutex);
        this->startCondition.wait(lk, [this]{ return this->started; });
        if (this->started) {
            cout << "device opened, starting capture" << endl;
            if (!this->openKinect2Device()) {
                exit(-1);
            }
            // If we get here we've shut down.
            lk.unlock();

            /* TODO: Fix this, there's a memory leak somewhere in libfreenect2 and we should be able to avoid it and not
             * have to bounce the process every time, but for now this is the cleaner way to do it and let systemd
             * restart us. */
            exit(0);
        }
    }
}

void Kinect2Pipe::writeBlankFrame() {
    memset(this->imageBuffer, 0x10, YUV_BUFFER_Y_LEN);
    memset(this->imageBuffer + YUV_BUFFER_Y_LEN, 0, YUV_BUFFER_UV_LEN * 2);
    write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN);
}
