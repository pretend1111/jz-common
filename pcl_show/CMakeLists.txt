cmake_minimum_required(VERSION 2.6 FATAL_ERROR)  
  
project(MY_GRAND_PROJECT)  
  
find_package(PCL 1.3 REQUIRED)  
  
# 将PCL的包含目录添加到项目的包含目录列表中，以便能够找到PCL的头文件  
include_directories(${PCL_INCLUDE_DIRS})  
  
# 将PCL的库目录添加到链接器搜索路径中  
link_directories(${PCL_LIBRARY_DIRS})  
  
add_definitions(${PCL_DEFINITIONS})  
  
# 添加一个可执行文件目标，名为pcd_write_test，源文件为pcd_write.cpp  
add_executable(pcd_write_test pcd_write.cpp)
add_executable(pcd_show_test pcd_show.cpp)  
  
# 将PCL的库链接到pcd_write_test可执行文件目标。这里${PCL_LIBRARIES}包含了PCL及其依赖的所有库  
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})
target_link_libraries(pcd_show_test ${PCL_LIBRARIES})
