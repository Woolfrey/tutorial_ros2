cmake_minimum_required(VERSION 3.8)
project(tutorial_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                         DEPENDENCIES                                             #
####################################################################################################
include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(${PROJECT_NAME} REQUIRED)

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
add_executable(action_server
    src/action_server.cpp
    src/HaikuActionServer.cpp
)

ament_target_dependencies(action_server
    "rclcpp"
    "rclcpp_action"
    "std_msgs"
    ${PROJECT_NAME}
)

add_executable(action_client
    src/action_client.cpp
    src/HaikuActionClient.cpp
)

ament_target_dependencies(action_client
    "rclcpp"
    "rclcpp_action"
    "std_msgs"
    ${PROJECT_NAME}
)

install(TARGETS
        action_server
        action_client
        DESTINATION lib/${PROJECT_NAME})
        
ament_package()
