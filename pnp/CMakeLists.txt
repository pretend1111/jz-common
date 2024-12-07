cmake_minimum_required(VERSION 3.11)

project(pnp)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(pnp pnp.cpp)

target_link_libraries( pnp ${OpenCV_LIBS} )