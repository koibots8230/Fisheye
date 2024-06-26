cmake_minimum_required(VERSION 3.2)

project(Vision)
find_package(OpenCV REQUIRED)
find_package(wpilib REQUIRED)

include_directories(${wpilib_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(fisheye main.cpp)

target_link_libraries(fisheye ${OpenCV_LIBS})
target_link_libraries(fisheye ntcore)