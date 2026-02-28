cmake_minimum_required(VERSION 3.10)
project(FisheyeVision)

set(CMAKE_CXX_STANDARD 17)

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Source files
set(SOURCES
        src/Fisheye.cpp
        src/Camera.cpp
        src/Utils.cpp
        src/main.cpp
)

add_executable(FisheyeVision ${SOURCES})

# Link OpenCV
target_link_libraries(FisheyeVision ${OpenCV_LIBS})