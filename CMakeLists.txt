cmake_minimum_required(VERSION 3.8)
project(tutorial_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                         DEPENDENCIES                                             #
####################################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

####################################################################################################
#                                       GENERATE INTERFACES                                        #
####################################################################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Haiku.action"
  DEPENDENCIES std_msgs
)

####################################################################################################
#                                          EXECUTABLES                                             #
####################################################################################################

ament_package()
