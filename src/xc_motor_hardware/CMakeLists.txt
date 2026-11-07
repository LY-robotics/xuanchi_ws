cmake_minimum_required(VERSION 3.16)
project(xc_motor_hardware LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/motor_protocol.cpp
  src/joint_conversion.cpp
  src/motor_config.cpp
  src/socketcan_transport.cpp
  src/sanpo_can_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp rclcpp_lifecycle)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
pluginlib_export_plugin_description_file(hardware_interface xc_motor_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config docs examples DESTINATION share/${PROJECT_NAME})
install(FILES xc_motor_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_protocol_conversion test/test_protocol_conversion.cpp src/motor_protocol.cpp src/joint_conversion.cpp)
  target_include_directories(test_protocol_conversion PRIVATE include)
  add_test(NAME test_protocol_conversion COMMAND test_protocol_conversion)

  add_executable(test_config test/test_config.cpp src/motor_config.cpp src/motor_protocol.cpp)
  target_include_directories(test_config PRIVATE include)
  target_link_libraries(test_config yaml-cpp)
  add_test(NAME test_config COMMAND test_config ${CMAKE_CURRENT_SOURCE_DIR}/config/right_arm.yaml)
endif()
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle yaml_cpp_vendor)
ament_package()
