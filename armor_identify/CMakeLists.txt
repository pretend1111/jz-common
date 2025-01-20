cmake_minimum_required(VERSION 3.8)
project(armor_identify)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(Torch REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(show_node src/show_node.cpp)
add_executable(traditional_node src/traditional_node.cpp)
add_executable(yolo_node src/yolo_node.cpp)

# Link the executable to the rclcpp library
ament_target_dependencies(show_node
  rclcpp
  OpenCV
  std_msgs
)

ament_target_dependencies(traditional_node
  rclcpp
  OpenCV
  std_msgs
)

ament_target_dependencies(yolo_node
  rclcpp
  OpenCV
  std_msgs
)

# Install the executable
install(TARGETS
  show_node
  traditional_node
  yolo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
