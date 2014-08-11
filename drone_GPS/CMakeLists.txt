#
# CMakeLists.txt
# This file is part of drone_GPS
# Author: Juan Pablo Rodríguez 
# Info: j_rodriguezg@javeriana.edu.co   
#

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
FIND_PACKAGE( OpenCV REQUIRED )
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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

include_directories (${OpenCV_INCLUDE_DIRS})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


#----- Headers ------
#set(GPS_HEADER_FILES src/UTM_converter.h)
#rosbuild_add_executable(UTM_converter src/UTM_converter.cpp ${GPS_HEADER_FILES})

rosbuild_add_executable(GPS src/UTM_converter.cpp)
rosbuild_add_executable(GPS_test src/GPS_test.cpp)

target_link_libraries(GPS ${OpenCV_LIBS})
target_link_libraries(GPS_test ${OpenCV_LIBS})

