cmake_minimum_required(VERSION 3.22)
project(edgepick_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/command_gateway.cpp
  src/mock_transport.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(command_gateway_test test/command_gateway_test.cpp)
  target_link_libraries(command_gateway_test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
