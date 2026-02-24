#ifndef kinect2pipe_IR_kinect2pipe_IR_H
#define kinect2pipe_IR_kinect2pipe_IR_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>

using namespace std;
using namespace libfreenect2;

#define KINECT2_IMAGE_WIDTH  512
#define KINECT2_IMAGE_HEIGHT 424

#define OUTPUT_WIDTH  512
#define OUTPUT_HEIGHT 424

#define YUV_BUFFER_Y_LEN   (OUTPUT_WIDTH * OUTPUT_HEIGHT)
#define YUV_BUFFER_UV_LEN  ((OUTPUT_WIDTH / 2) * (OUTPUT_HEIGHT / 2))
#define YUV_BUFFER_LEN     (YUV_BUFFER_Y_LEN + (YUV_BUFFER_UV_LEN * 2))

#define IR_MAX_VALUE 65535.0f

// If no frame has been successfully delivered to a streaming consumer for
// this many seconds the process will shut down.  This prevents passive
// openers such as Chrome (which enumerate the device but never call
// VIDIOC_STREAMON) from keeping the Kinect running indefinitely.
#define IDLE_SHUTDOWN_SECONDS 30

class kinect2pipe_IR {
public:
    explicit kinect2pipe_IR();
    void openLoopback(const char* loopbackDev);
    void setBackupDevice(const char* dev);
    void run();
    void shutdown();

private:
    Freenect2        freenect2;
    FrameMap         frames;
    int              v4l2Device;

    std::string        backupDevPath;

    struct SwsContext* sws;
    float*             normBuf;

    uint8_t* srcPtr[4]{};
    int      srcStride[4]{};
    uint8_t* dstPtr[4]{};
    int      dstStride[4]{};

    uint8_t* imageBuffer;

    std::thread        watcherThread;
    mutex              cvMutex;
    condition_variable cv;
    bool               started;    // first consumer opened
    bool               shouldStop; // last consumer gone or signal
    std::atomic<bool>  cleanupComplete; // set when device shutdown finished

    // Time of the last write() that successfully delivered a frame to a
    // streaming consumer.  Used to detect "passive" openers (e.g. Chrome
    // enumerating devices) that never call VIDIOC_STREAMON.
    std::chrono::steady_clock::time_point lastSuccessWrite;

    bool openV4L2LoopbackDevice(const char* loopbackDev, int width, int height);
    bool openInotifyWatcher(const char* loopbackDev);
    bool openKinect2Device();
    bool openBackupDevice();
    bool handleFrame(Frame* frame);
    void inotifyWatcher(const char* loopbackDev);
    void writeBlankFrame();
};

#endif // kinect2pipe_IR_kinect2pipe_IR_H