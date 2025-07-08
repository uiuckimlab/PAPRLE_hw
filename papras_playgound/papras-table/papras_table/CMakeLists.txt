cmake_minimum_required(VERSION 3.8)
project(papras_table)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles # maybe some unrelated packes included
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  urdf
  kdl_parser
  orocos_kdl_vendor
  rosidl_default_generators
  teleop_msgs
  yaml-cpp
  pinocchio
)


find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY description launch config
  DESTINATION share/${PROJECT_NAME}
)



ament_export_include_directories(
  include
)


ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
