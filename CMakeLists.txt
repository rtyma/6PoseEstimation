cmake_minimum_required(VERSION 3.8)
project(6PoseEstmation)

FIND_PACKAGE(OpenCV 3 REQUIRED core flann xfeatures2d features2d imgcodecs imgproc highgui ml opencv_cudaarithm cudafilters cudawarping cudaimgproc
        cudafeatures2d cudaoptflow cudabgsegm
        cudastereo cudalegacy cudaobjdetect viz)

set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES main.cpp src/Descriptor.cpp inc/Descriptor.h src/Visualizer.cpp inc/Visualizer.h src/Pose.cpp inc/Pose.h)
add_executable(6PoseEstmation ${SOURCE_FILES})

set ( DEPENDENCIES ${OpenCV_LIBS} )
target_link_libraries( ${target}6PoseEstmation ${OPENCV_LINKER_LIBS} ${OPENCV_CPP_SAMPLES_REQUIRED_DEPS} ${OpenCV_LIBS})