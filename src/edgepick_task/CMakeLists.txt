cmake_minimum_required(VERSION 3.22)
project(edgepick_task)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/grasp_state_machine.cpp
  src/task_event_io.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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
  TARGETS task_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(grasp_state_machine_test test/grasp_state_machine_test.cpp)
  target_link_libraries(grasp_state_machine_test ${PROJECT_NAME})

  ament_add_gtest(task_event_io_test test/task_event_io_test.cpp)
  target_link_libraries(task_event_io_test ${PROJECT_NAME})
endif()

ament_export_dependencies(diagnostic_msgs rclcpp std_msgs)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
