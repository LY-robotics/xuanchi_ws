cmake_minimum_required(VERSION 3.8)
project(xc_arm_api_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(xc_arm_interface REQUIRED)



add_executable(xc_arm_commander src/commander_template.cpp)
ament_target_dependencies(xc_arm_commander rclcpp moveit_ros_planning_interface example_interfaces xc_arm_interface)



install(TARGETS
  xc_arm_commander
  DESTINATION lib/${PROJECT_NAME})



ament_package()
