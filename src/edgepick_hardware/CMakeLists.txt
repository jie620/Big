cmake_minimum_required(VERSION 3.22)
project(edgepick_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME}
  src/command_gateway.cpp
  src/mock_system_interface.cpp
  src/mock_transport.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface edgepick_hardware_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(command_gateway_test test/command_gateway_test.cpp)
  target_link_libraries(command_gateway_test ${PROJECT_NAME})

  ament_add_gtest(mock_system_interface_test test/mock_system_interface_test.cpp)
  target_link_libraries(mock_system_interface_test ${PROJECT_NAME})
  ament_target_dependencies(mock_system_interface_test hardware_interface rclcpp)
endif()

ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
