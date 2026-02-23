#ifndef kinect2pipe_IR_kinect2pipe_IR_H
#define kinect2pipe_IR_kinect2pipe_IR_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace std;
using namespace libfreenect2;

// Native IR resolution of Kinect v2
#define KINECT2_IMAGE_WIDTH  512
#define KINECT2_IMAGE_HEIGHT 424

#define OUTPUT_WIDTH  512
#define OUTPUT_HEIGHT 424

#define YUV_BUFFER_Y_LEN   (OUTPUT_WIDTH * OUTPUT_HEIGHT)
#define YUV_BUFFER_UV_LEN  ((OUTPUT_WIDTH / 2) * (OUTPUT_HEIGHT / 2))
#define YUV_BUFFER_LEN     (YUV_BUFFER_Y_LEN + (YUV_BUFFER_UV_LEN * 2))

// libfreenect2 IR frames are 32-bit float, values 0–65535.
// We normalise to 0.0–1.0 before passing to sws_scale (AV_PIX_FMT_GRAYF32).
#define IR_MAX_VALUE 65535.0f

class kinect2pipe_IR {
public:
    explicit kinect2pipe_IR();
    void openLoopback(const char* loopbackDev);
    void run();

private:
    Freenect2  freenect2;
    FrameMap   frames;
    int        v4l2Device;

    struct SwsContext* sws;

    // Normalised float buffer (values in 0.0–1.0) used as sws source
    float*    normBuf;

    uint8_t*  srcPtr[4]{};
    int       srcStride[4]{};
    uint8_t*  dstPtr[4]{};
    int       dstStride[4]{};

    uint8_t*  imageBuffer;

    std::thread             watcherThread;
    int                     openCount;
    mutex                   startMutex;
    condition_variable      startCondition;
    bool                    started;

    bool openV4L2LoopbackDevice(const char* loopbackDev, int width, int height);
    bool openProcMonitorV4L2LoopbackDevice(const char* loopbackDev);
    bool openKinect2Device();
    bool handleFrame(Frame* frame);
    void procMonitorV4L2LoopbackDevice(const char* loopbackDev);
    void writeBlankFrame();
};

#endif // kinect2pipe_IR_kinect2pipe_IR_H