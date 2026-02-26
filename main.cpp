#include "kinect2pipe_IR.h"
#include <vector>
#include <cstring>

/**
 * Main is the entry point for the application. It takes a required argument which is the path to the v4l2loopback
 * device to write to, and an optional second argument which is the path to a backup V4L2 capture device used when
 * the Kinect 2 is unavailable.
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return Exit status
 */
int main(int argc, char** argv) {
    bool hwaccel = false;
    std::vector<char*> positional;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--hwaccel") == 0) {
            hwaccel = true;
        } else {
            positional.push_back(argv[i]);
        }
    }

    if (positional.size() < 1 || positional.size() > 2) {
        printf(
            "usage: kinect2pipe_IR [--hwaccel] [path to v4l2loopback device] "
            "[optional: path to backup v4l2 capture device]\n");
        exit(-1);
    }

    auto* pipe = new kinect2pipe_IR();
    pipe->setHwAccel(hwaccel);
    pipe->openLoopback(positional[0]);
    if (positional.size() == 2) {
        pipe->setBackupDevice(positional[1]);
    }
    pipe->run();
    return 0;
}