#include "kinect2pipe_IR.h"

/**
 * Main is the entry point for the application. It takes a required argument which is the path to the v4l2loopback
 * device to write to, and an optional second argument which is the path to a backup V4L2 capture device used when
 * the Kinect 2 is unavailable.
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return Exit status
 */
int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        printf("usage: kinect2pipe_IR [path to v4l2loopback device] [optional: path to backup v4l2 capture device]\n");
        exit(-1);
    }

    auto* pipe = new kinect2pipe_IR();
    pipe->openLoopback(argv[1]);
    if (argc == 3) {
        pipe->setBackupDevice(argv[2]);
    }
    pipe->run();
    return 0;
}