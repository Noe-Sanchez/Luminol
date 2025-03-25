cmake_minimum_required(VERSION 3.8)
project(luminol_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include/cpp-httplib)

add_executable(luminol_bridge src/luminol_bridge.cpp)
ament_target_dependencies(luminol_bridge rclcpp std_msgs geometry_msgs)

ament_python_install_package(${PROJECT_NAME})

#install(PROGRAMS
#  path/script.py
#  DESTINATION lib/${PROJECT_NAME}
#)

install(TARGETS
  luminol_bridge 
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
