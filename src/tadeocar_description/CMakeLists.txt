cmake_minimum_required(VERSION 3.8)
project(tadeocar_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  test
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # The URDF/SDF consistency test: wheel radius, sensor mounts, joint limits.
  # It existed and passed, but was never registered, so colcon test did not
  # run it and a change that broke the geometry would have gone unnoticed.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_sdf_match test/test_urdf_sdf_match.py
    TIMEOUT 120)
endif()

ament_package()
