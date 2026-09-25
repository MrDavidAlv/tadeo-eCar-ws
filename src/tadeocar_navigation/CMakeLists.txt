cmake_minimum_required(VERSION 3.8)
project(tadeocar_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Two of the common linters contradict conventions held everywhere else in
  # this repository, so they fail here and nowhere else only because this is
  # the one package that runs them. ament_copyright wants a header in every
  # file; the licence lives in LICENSE and in each package.xml instead.
  # ament_pep257 enforces D213, summary on the second line of a docstring;
  # every docstring in the workspace puts it on the first. flake8, lint_cmake
  # and xmllint stay on.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
