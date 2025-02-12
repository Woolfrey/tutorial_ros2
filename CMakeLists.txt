cmake_minimum_required(VERSION 3.8)
project(tutorial_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                         DEPENDENCIES                                             #
####################################################################################################

include_directories(include
                    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(${PROJECT_NAME} REQUIRED)

####################################################################################################
#                                    GENERATE SERVICE INTERFACES                                   #
####################################################################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Haiku.srv"
  DEPENDENCIES std_msgs
)

####################################################################################################
#                                          EXECUTABLES                                             #
####################################################################################################

add_executable(service src/service.cpp src/HaikuService.cpp)

ament_target_dependencies(service
    rclcpp
    std_msgs
    ${PROJECT_NAME}
)

install(TARGETS
    service
    DESTINATION lib/${PROJECT_NAME})

ament_package()
