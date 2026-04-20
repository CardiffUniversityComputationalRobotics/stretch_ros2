cmake_minimum_required(VERSION 3.8)
project(stretch_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create StretchDiffDrive
add_library(StretchDiffDrive
  SHARED
  src/StretchDiffDrive.cc
  src/StretchDiffDrivePrivate.cc
)
target_include_directories(
  StretchDiffDrive PRIVATE include
)
ament_target_dependencies(StretchDiffDrive
  rclcpp
  gz-sim${GZ_SIM_VER}
  nav_msgs
  gz-math7
  tf2_ros
  geometry_msgs
)
install(
  TARGETS StretchDiffDrive
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    launch
    worlds
    urdf
    meshes
  DESTINATION 
    share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
