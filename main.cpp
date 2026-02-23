#include "kinect2pipe_IR.h"

/**
 * Main is the entry point for the application. It takes a single argument which should be the path to the v4l2loopback
 * device to watch.
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return Exit status
 */
int main(int argc, char** argv) {
    if (argc != 2) {
        printf("usage: kinect2pipe_IR [path to v4l2loopback device]\n");
        exit(-1);
    }

    auto* pipe = new kinect2pipe_IR();
    pipe->openLoopback(argv[1]);
    pipe->run();
    return 0;
}