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

# option allowing the installer to specify a backup path that will
# be injected into the systemd unit's ExecStart line. This can be set
# at configuration time with -DBACKUP_PATH=/path when calling cmake
option(BACKUP_PATH "path passed to kinect2pipe_IR via --backup" "")

# option to control the first argument (loopback device) used by the
# systemd unit. The default is /dev/video11 but can be
# overridden precisely the same way as BACKUP_PATH
option(LOOPBACK_DEVICE "v4l2loopback device path for service ExecStart" "/dev/video11")

# build the actual strings we will substitute into the service file;
# both the loopback device and backup argument are injected. The loopback
# argument always has a value (defaults to /dev/video11), whereas the
# backup argument is left empty when unset so the ExecStart line remains clean.
set(LOOPBACK_ARG "${LOOPBACK_DEVICE}")

if(BACKUP_PATH)
    set(BACKUP_ARG "${BACKUP_PATH}")
else()
    set(BACKUP_ARG "")
endif()

# configure systemd service unit so that the executable path and arguments
# reflect the current configuration. The @CMAKE_INSTALL_PREFIX@,
# @LOOPBACK_ARG@ and @BACKUP_ARG@ placeholders are substituted when
# configure_file is invoked during the generate step.
configure_file(
    "${CMAKE_SOURCE_DIR}/contrib/systemd/kinect2pipe_IR.service"
    "${CMAKE_CURRENT_BINARY_DIR}/kinect2pipe_IR.service"
    @ONLY
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/kinect2pipe_IR.service"
    DESTINATION "$ENV{HOME}/.config/systemd/user"
)