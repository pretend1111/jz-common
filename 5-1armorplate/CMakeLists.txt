cmake_minimum_required(VERSION 3.11)

project(shibie)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(shibie shibie.cpp)
add_executable(ceshi ceshi.cpp)
add_executable(number number.cpp)

target_link_libraries( shibie ${OpenCV_LIBS} )
target_link_libraries( ceshi ${OpenCV_LIBS} )
target_link_libraries( number ${OpenCV_LIBS} )