cmake_minimum_required(VERSION 3.22)
project(edgepick_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(bringup_config_test test/test_bringup_config.py
    ENV
      EDGE_PICK_BRINGUP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 30
  )
endif()

ament_package()
