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

# options for systemd User/Group substitution. These can be set at
# configuration time with -DSERVICE_USER=username -DSERVICE_GROUP=group
# Default SERVICE_USER is taken from the build environment $ENV{USER}
set(SERVICE_USER "$ENV{USER}" CACHE STRING "User for systemd service (replaces @SERVICE_USER@ in service file)")
set(SERVICE_GROUP "" CACHE STRING "Group for systemd service (replaces @SERVICE_GROUP@ in service file)")
if(NOT SERVICE_GROUP)
    set(SERVICE_GROUP "${SERVICE_USER}")
endif()

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
    DESTINATION "/etc/systemd/system"
)

# attempt to reload and enable the systemd unit automatically at install time.
# These commands require appropriate privileges (typically root). If systemctl
# is not available or the commands fail, a warning will be emitted but the
# install will not be aborted.
find_program(SYSTEMCTL_EXECUTABLE systemctl)
if(SYSTEMCTL_EXECUTABLE)
    install(CODE "execute_process(COMMAND ${SYSTEMCTL_EXECUTABLE} daemon-reload
        RESULT_VARIABLE _sysctl_res1
        OUTPUT_QUIET ERROR_QUIET)
    if(NOT _sysctl_res1 EQUAL 0)
        message(WARNING \\\"systemctl daemon-reload failed with ${_sysctl_res1}\\\")
    endif()
    execute_process(COMMAND ${SYSTEMCTL_EXECUTABLE} enable --now kinect2pipe_IR
        RESULT_VARIABLE _sysctl_res2
        OUTPUT_QUIET ERROR_QUIET)
    if(NOT _sysctl_res2 EQUAL 0)
        message(WARNING \\\"systemctl enable --now kinect2pipe_IR failed with ${_sysctl_res2}\\\")
    endif()
    execute_process(COMMAND ${SYSTEMCTL_EXECUTABLE} restart kinect2pipe_IR
        RESULT_VARIABLE _sysctl_res3
        OUTPUT_QUIET ERROR_QUIET)
    if(NOT _sysctl_res3 EQUAL 0)
        message(WARNING \\\"systemctl restart kinect2pipe_IR failed with ${_sysctl_res3}\\\")
    endif()")
else()
    message(WARNING "systemctl not found; skipping service daemon-reload/enable/restart")
endif()
