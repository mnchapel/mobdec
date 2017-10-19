#CMake arguments for debug mode : -DCMAKE_BUILD_TYPE=Debug

cmake_minimum_required(VERSION 2.8)
project(project_mobdec)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
message("CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

# add OpenCV ----------------------
SET("OpenCV_DIR" "/usr/local/opencv/3.3/share/OpenCV")
find_package(OpenCV REQUIRED)

# add CUDA ------------------------
find_package(CUDA REQUIRED)

# add LDOF ------------------------
link_directories("${CMAKE_SOURCE_DIR}/libs/")

include_directories(${OpenCV_INCLUDE_DIRS} "${CMAKE_SOURCE_DIR}/src/")

set(LIBS
    ${LIBS}
    "ldof"
    "opencv_core"
    "opencv_highgui"
    "opencv_ml"
    "opencv_xfeatures2d"
    "opencv_features2d"
    "opencv_flann"
    "opencv_imgproc"
    "opencv_video"
    "opencv_calib3d"
    "opencv_imgcodecs")

file(GLOB_RECURSE SRC src/*/*.cpp)
file(GLOB_RECURSE HDR src/*/*.h)
add_executable(mobdec ${SRC} ${HDR}) # add HDR to executable to show them in the project explorer of qt creator

target_link_libraries(mobdec ${LIBS})
