cmake_minimum_required(VERSION 3.22)
project(edgepick_perception)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(edgepick_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/perception_metrics.cpp
  src/rgbd_projection.cpp
  src/target_detection_selection.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
)

add_executable(rgbd_target_candidate_node
  src/rgbd_target_candidate_node.cpp
)
target_link_libraries(rgbd_target_candidate_node ${PROJECT_NAME})
ament_target_dependencies(rgbd_target_candidate_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(detected_target_candidate_node
  src/detected_target_candidate_node.cpp
)
target_link_libraries(detected_target_candidate_node ${PROJECT_NAME})
ament_target_dependencies(detected_target_candidate_node
  edgepick_interfaces
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(mock_detector_node
  src/mock_detector_node.cpp
)
ament_target_dependencies(mock_detector_node
  edgepick_interfaces
  rclcpp
)

add_executable(perception_metrics_node
  src/perception_metrics_node.cpp
)
target_link_libraries(perception_metrics_node ${PROJECT_NAME})
ament_target_dependencies(perception_metrics_node
  builtin_interfaces
  edgepick_interfaces
  geometry_msgs
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
  TARGETS
    detected_target_candidate_node
    mock_detector_node
    perception_metrics_node
    rgbd_target_candidate_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rgbd_projection_test test/rgbd_projection_test.cpp)
  target_link_libraries(rgbd_projection_test ${PROJECT_NAME})

  ament_add_gtest(target_detection_selection_test test/target_detection_selection_test.cpp)
  target_link_libraries(target_detection_selection_test ${PROJECT_NAME})

  ament_add_gtest(perception_metrics_test test/perception_metrics_test.cpp)
  target_link_libraries(perception_metrics_test ${PROJECT_NAME})
endif()

ament_export_dependencies(builtin_interfaces edgepick_interfaces sensor_msgs)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
