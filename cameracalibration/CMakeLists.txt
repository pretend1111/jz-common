cmake_minimum_required(VERSION 3.11)

project(4_2)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(4_2 4_2.cpp)

target_link_libraries( 4_2 ${OpenCV_LIBS} )