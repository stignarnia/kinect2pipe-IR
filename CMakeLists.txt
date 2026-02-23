cmake_minimum_required(VERSION 3.16)
project(kinect2pipe_IR)

set(CMAKE_CXX_STANDARD 14)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libswscale REQUIRED IMPORTED_TARGET libswscale)
pkg_check_modules(freenect2 REQUIRED IMPORTED_TARGET freenect2)

add_executable(kinect2pipe_IR kinect2pipe_IR.cpp main.cpp)
target_include_directories(kinect2pipe_IR PRIVATE ${freenect2_INCLUDE_DIRS} ${libswscale_INCLUDE_DIRS})
target_link_directories(kinect2pipe_IR PRIVATE ${freenect2_LIBRARY_DIRS} {libswscale_LIBRARY_DIRS})
target_link_libraries(kinect2pipe_IR ${freenect2_LIBRARIES} ${libswscale_LIBRARIES} pthread)

INSTALL(TARGETS kinect2pipe_IR DESTINATION ${CMAKE_INSTALL_BINDIR})
INSTALL(FILES "${CMAKE_SOURCE_DIR}/contrib/systemd/kinect2pipe_IR.service" DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/systemd/system)