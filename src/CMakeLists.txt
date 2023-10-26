set (CMAKE_CXX_STANDARD 11)
project(apriltag)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3 -std=c++11 ")

find_package(OpenCV REQUIRED)

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")
link_directories(${apriltag_LIBDIR})


include_directories(include
  ${apriltag_INCLUDE_DIRS}
)

add_executable(vision
main.cpp
)

target_link_libraries(vision ${apriltag_LIBRARIES} ${OpenCV_LIBS})