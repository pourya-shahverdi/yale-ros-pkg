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
rosbuild_add_boost_directories()
rosbuild_add_executable(viseme_relay src/viseme_relay.cc)
rosbuild_link_boost(viseme_relay thread)
rosbuild_add_executable(expression_relay src/expression_relay.cc)
rosbuild_link_boost(expression_relay thread)
rosbuild_add_executable(ik_relay src/ik_relay.cc)
rosbuild_link_boost(ik_relay thread)
rosbuild_add_executable(lookat_relay src/lookat_relay.cc)
rosbuild_link_boost(lookat_relay thread)
rosbuild_add_executable(blink_relay src/blink_relay.cc)
rosbuild_link_boost(blink_relay thread)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
