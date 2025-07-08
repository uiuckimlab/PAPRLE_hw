################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(teleop_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS 
             roscpp 
             rospy 
             std_msgs 
             sensor_msgs 
             geometry_msgs 
             message_generation)

################################################################################
# Msg file
################################################################################
add_message_files(
  FILES
  "EEFTransforms.msg"
)

add_service_files(
  FILES
  "SetBasePose.srv"
  "SetEEFNames.srv"
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
  CATKIN_DEPENDS
    message_runtime
)

catkin_package(
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    sensor_msgs
    std_msgs
)

