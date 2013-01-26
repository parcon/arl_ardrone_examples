cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(led_dance src/control_leds.cpp)
rosbuild_add_executable(ardrone_test src/test_fly.cpp)
rosbuild_add_executable(ardrone_test_back_forth src/fly_back_forth.cpp)
rosbuild_add_executable(reset src/reset.cpp)
rosbuild_add_executable(land src/land.cpp)
rosbuild_add_executable(takeoff src/takeoff.cpp)
rosbuild_add_executable(test_find_K src/test_fly_find_K.cpp)
rosbuild_add_executable(joy_test_find_K src/fly_from_joy_test_K.cpp)
#rosbuild_add_executable(pid_point src/pid_to_point.cpp)
rosbuild_add_executable(fly_joy src/fly_from_joy.cpp)
#target_link_libraries(example ${PROJECT_NAME})
