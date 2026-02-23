cmake_minimum_required(VERSION 3.16)
project(kinect2pipe_IR)

set(CMAKE_CXX_STANDARD 14)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libswscale REQUIRED IMPORTED_TARGET libswscale)
pkg_check_modules(freenect2 REQUIRED IMPORTED_TARGET freenect2)

add_executable(kinect2pipe_IR kinect2pipe_IR.cpp main.cpp)

target_link_libraries(kinect2pipe_IR PRIVATE
    PkgConfig::freenect2
    PkgConfig::libswscale
    pthread
)

# installation rules -------------------------------------------------------
install(TARGETS kinect2pipe_IR
    RUNTIME DESTINATION bin
)

# configure systemd service unit so that the executable path matches the
# installation prefix. The @CMAKE_INSTALL_PREFIX@ placeholder is substituted
# when configure_file is invoked during the generate step.
configure_file(
    "${CMAKE_SOURCE_DIR}/contrib/systemd/kinect2pipe_IR.service"
    "${CMAKE_CURRENT_BINARY_DIR}/kinect2pipe_IR.service"
    @ONLY
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/kinect2pipe_IR.service"
    DESTINATION "$ENV{HOME}/.config/systemd/user"
)
