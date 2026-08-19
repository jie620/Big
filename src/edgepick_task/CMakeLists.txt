cmake_minimum_required(VERSION 3.22)
project(edgepick_task)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/grasp_target_builder.cpp
  src/grasp_state_machine.cpp
  src/mock_task_script.cpp
  src/moveit_action_event_mapper.cpp
  src/task_event_io.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
)

add_executable(task_node
  src/task_node.cpp
)
target_link_libraries(task_node ${PROJECT_NAME})
ament_target_dependencies(task_node
  diagnostic_msgs
  rclcpp
  std_msgs
)

add_executable(mock_task_driver_node
  src/mock_task_driver_node.cpp
)
target_link_libraries(mock_task_driver_node ${PROJECT_NAME})
ament_target_dependencies(mock_task_driver_node
  rclcpp
  std_msgs
)

add_executable(moveit_action_adapter_node
  src/moveit_action_adapter_node.cpp
)
target_link_libraries(moveit_action_adapter_node ${PROJECT_NAME})
ament_target_dependencies(moveit_action_adapter_node
  moveit_msgs
  rclcpp
  rclcpp_action
  std_msgs
)

add_executable(grasp_target_builder_node
  src/grasp_target_builder_node.cpp
)
target_link_libraries(grasp_target_builder_node ${PROJECT_NAME})
ament_target_dependencies(grasp_target_builder_node
  geometry_msgs
  rclcpp
)

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
install(
  TARGETS
    task_node
    mock_task_driver_node
    moveit_action_adapter_node
    grasp_target_builder_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(grasp_target_builder_test test/grasp_target_builder_test.cpp)
  target_link_libraries(grasp_target_builder_test ${PROJECT_NAME})

  ament_add_gtest(grasp_state_machine_test test/grasp_state_machine_test.cpp)
  target_link_libraries(grasp_state_machine_test ${PROJECT_NAME})

  ament_add_gtest(task_event_io_test test/task_event_io_test.cpp)
  target_link_libraries(task_event_io_test ${PROJECT_NAME})

  ament_add_gtest(mock_task_script_test test/mock_task_script_test.cpp)
  target_link_libraries(mock_task_script_test ${PROJECT_NAME})

  ament_add_gtest(moveit_action_event_mapper_test test/moveit_action_event_mapper_test.cpp)
  target_link_libraries(moveit_action_event_mapper_test ${PROJECT_NAME})
endif()

ament_export_dependencies(diagnostic_msgs geometry_msgs moveit_msgs rclcpp rclcpp_action std_msgs)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
