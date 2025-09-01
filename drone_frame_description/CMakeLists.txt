cmake_minimum_required(VERSION 3.8)
project(drone_frame_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  meshes
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/thruster_commander.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install resource files
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
