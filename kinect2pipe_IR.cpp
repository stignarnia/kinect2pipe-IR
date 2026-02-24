#include <iostream>
#include <libfreenect2/logger.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <linux/videodev2.h>
#include <sys/inotify.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <thread>
#include <cstring>
#include <vector>
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

void kinect2pipe_IR::setBackupDevice(const char* dev) {
    this->backupDevPath = dev;
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

    // When a backup device is configured use a short timeout so a USB disconnect
    // is detected within ~0.5 s (5 × 100 ms).  Without backup the original
    // 1-second timeout is kept to preserve the existing behaviour.
    const int  timeoutMs  = this->backupDevPath.empty() ? 1000 : 100;
    const int  maxMissed  = 5;
    int        missedCount = 0;

    while (true) {
        {
            unique_lock<mutex> lk(this->cvMutex);
            if (this->shouldStop) break;
        }

        if (!listener.waitForNewFrame(frames, timeoutMs)) {
            if (!this->backupDevPath.empty() && ++missedCount >= maxMissed) {
                cerr << "kinect2 device not responding, switching to backup" << endl;
                break;
            }
            continue;
        }
        missedCount = 0;

        if (!this->handleFrame(frames[Frame::Ir])) {
            listener.release(frames);
            break;
        }

        listener.release(frames);
    }

    cout << "stopping kinect2 IR stream" << endl;
    this->writeBlankFrame();
    // Run stop/close in a detached thread: libfreenect2 can block indefinitely
    // when the USB device has been yanked or when the system is shutting down.
    // Detaching lets the main thread proceed to the backup device (or exit)
    // without hanging, and the OS will reap the thread when the process exits.
    // `dev` is managed by libfreenect2 internally; close() is its release call,
    // so the pointer remains valid until that call completes in the thread.
    thread([dev]{
        try { dev->stop(); dev->close(); }
        catch (...) { cerr << "kinect2 cleanup thread caught exception" << endl; }
    }).detach();
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

bool kinect2pipe_IR::openBackupDevice() {
    cout << "switching to backup device: " << this->backupDevPath << endl;

    int fd = open(this->backupDevPath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        cerr << "failed to open backup device: " << strerror(errno) << endl;
        return false;
    }

    // Try to negotiate YUYV; if the driver refuses, accept whatever it gives us.
    struct v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        cerr << "backup device: VIDIOC_G_FMT failed: " << strerror(errno) << endl;
        close(fd);
        return false;
    }
    uint32_t wantedFmt = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.pixelformat = wantedFmt;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        cerr << "backup device: driver refused YUYV, using native format" << endl;
    }
    ioctl(fd, VIDIOC_G_FMT, &fmt);   // re-read what was actually set

    int      capWidth  = (int)fmt.fmt.pix.width;
    int      capHeight = (int)fmt.fmt.pix.height;
    uint32_t pixfmt    = fmt.fmt.pix.pixelformat;

    AVPixelFormat avfmt;
    switch (pixfmt) {
        case V4L2_PIX_FMT_YUYV:    avfmt = AV_PIX_FMT_YUYV422; break;
        case V4L2_PIX_FMT_UYVY:    avfmt = AV_PIX_FMT_UYVY422; break;
        case V4L2_PIX_FMT_GREY:    avfmt = AV_PIX_FMT_GRAY8;   break;
        case V4L2_PIX_FMT_YUV420:  avfmt = AV_PIX_FMT_YUV420P; break;
        case V4L2_PIX_FMT_NV12:    avfmt = AV_PIX_FMT_NV12;    break;
        case V4L2_PIX_FMT_BGR24:   avfmt = AV_PIX_FMT_BGR24;   break;
        case V4L2_PIX_FMT_RGB24:   avfmt = AV_PIX_FMT_RGB24;   break;
        default:
            cerr << "backup device: unsupported pixel format: "
                 << (char)(pixfmt & 0xff)         << (char)((pixfmt >> 8) & 0xff)
                 << (char)((pixfmt >> 16) & 0xff) << (char)((pixfmt >> 24) & 0xff)
                 << endl;
            close(fd);
            return false;
    }

    // Request MMAP capture buffers.
    struct v4l2_requestbuffers req{};
    req.count  = 2;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 1) {
        cerr << "backup device: VIDIOC_REQBUFS failed: " << strerror(errno) << endl;
        close(fd);
        return false;
    }

    struct BufInfo { void* start; size_t length; };
    std::vector<BufInfo> bufs(req.count, BufInfo{nullptr, 0});
    unsigned nbuf = req.count;
    for (unsigned i = 0; i < nbuf; i++) {
        struct v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            for (unsigned j = 0; j < i; j++) munmap(bufs[j].start, bufs[j].length);
            close(fd); return false;
        }
        bufs[i].length = buf.length;
        bufs[i].start  = mmap(nullptr, buf.length,
                               PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (bufs[i].start == MAP_FAILED) {
            for (unsigned j = 0; j < i; j++) munmap(bufs[j].start, bufs[j].length);
            close(fd); return false;
        }
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            for (unsigned j = 0; j <= i; j++) munmap(bufs[j].start, bufs[j].length);
            close(fd); return false;
        }
    }

    enum v4l2_buf_type btype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &btype) < 0) {
        cerr << "backup device: VIDIOC_STREAMON failed: " << strerror(errno) << endl;
        for (unsigned i = 0; i < nbuf; i++) munmap(bufs[i].start, bufs[i].length);
        close(fd);
        return false;
    }

    cout << "backup device stream started (" << capWidth << "x" << capHeight << ")" << endl;

    // Build a swscale context to convert the backup camera's frames to the
    // same YUV420P format at OUTPUT_WIDTH x OUTPUT_HEIGHT that the loopback
    // device expects.
    SwsContext* backupSws = sws_getContext(
        capWidth, capHeight, avfmt,
        OUTPUT_WIDTH, OUTPUT_HEIGHT, AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, nullptr, nullptr, nullptr);

    static const int BACKUP_SELECT_TIMEOUT_S = 1;

    bool ok = true;
    while (true) {
        {
            unique_lock<mutex> lk(this->cvMutex);
            if (this->shouldStop) break;
        }

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        struct timeval tv = {BACKUP_SELECT_TIMEOUT_S, 0};
        int r = select(fd + 1, &fds, nullptr, nullptr, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            ok = false; break;
        }
        if (r == 0) continue; // timeout – re-check shouldStop

        struct v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) { ok = false; break; }

        // Build per-plane pointers depending on whether the format is packed
        // or planar.
        uint8_t* base = (uint8_t*)bufs[buf.index].start;
        uint8_t* srcData[4]    = {};
        int      srcStrides[4] = {};
        switch (pixfmt) {
            case V4L2_PIX_FMT_YUYV:
            case V4L2_PIX_FMT_UYVY:
            case V4L2_PIX_FMT_BGR24:
            case V4L2_PIX_FMT_RGB24:
            case V4L2_PIX_FMT_GREY:
                srcData[0]    = base;
                srcStrides[0] = (pixfmt == V4L2_PIX_FMT_BGR24 ||
                                 pixfmt == V4L2_PIX_FMT_RGB24)
                                    ? capWidth * 3
                                    : (pixfmt == V4L2_PIX_FMT_GREY)
                                        ? capWidth
                                        : capWidth * 2;
                break;
            case V4L2_PIX_FMT_YUV420:
                srcData[0]    = base;
                srcData[1]    = base + capWidth * capHeight;
                srcData[2]    = base + capWidth * capHeight
                                     + (capWidth / 2) * (capHeight / 2);
                srcStrides[0] = capWidth;
                srcStrides[1] = capWidth / 2;
                srcStrides[2] = capWidth / 2;
                break;
            case V4L2_PIX_FMT_NV12:
                srcData[0]    = base;
                srcData[1]    = base + capWidth * capHeight;
                srcStrides[0] = capWidth;
                srcStrides[1] = capWidth;
                break;
            default:
                break;
        }

        sws_scale(backupSws, srcData, srcStrides, 0, capHeight,
                  this->dstPtr, this->dstStride);

        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) { ok = false; break; }

        if (write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN) < 0) {
            ok = false; break;
        }
    }

    sws_freeContext(backupSws);
    ioctl(fd, VIDIOC_STREAMOFF, &btype);
    for (unsigned i = 0; i < nbuf; i++) munmap(bufs[i].start, bufs[i].length);
    close(fd);
    return ok;
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
    bool kinectOk = this->openKinect2Device();

    // If shutdown was requested during capture, exit cleanly.
    {
        unique_lock<mutex> lk(this->cvMutex);
        if (this->shouldStop) exit(0);
    }

    // Fall back to the backup device when kinect failed to open or disconnected.
    if (!this->backupDevPath.empty()) {
        this->openBackupDevice();
        exit(0);
    }

    if (!kinectOk) exit(-1);
    exit(0);
}

void kinect2pipe_IR::writeBlankFrame() {
    memset(this->imageBuffer,                    0x10, YUV_BUFFER_Y_LEN);
    memset(this->imageBuffer + YUV_BUFFER_Y_LEN, 0x80, YUV_BUFFER_UV_LEN * 2);
    write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN);
}