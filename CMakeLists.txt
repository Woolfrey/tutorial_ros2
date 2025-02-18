cmake_minimum_required(VERSION 3.8)
project(tutorial_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                         DEPENDENCIES                                             #
####################################################################################################

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

####################################################################################################
#                                       GENERATE INTERFACES                                        #
####################################################################################################

rosidl_generate_interfaces(${PROJECT_NAME}
                           "srv/Haiku.srv"
                           DEPENDENCIES std_msgs)

####################################################################################################
#                                       BUILD TARGETS                                              #
####################################################################################################

ament_export_dependencies(rosidl_default_runtime)

add_library(${PROJECT_NAME}_interfaces INTERFACE)

target_link_libraries(${PROJECT_NAME}_interfaces INTERFACE
                      ${PROJECT_NAME}__rosidl_typesupport_cpp)

rosidl_get_typesupport_target(cpp_typesupport_target
                              ${PROJECT_NAME} rosidl_typesupport_cpp)

####################################################################################################
#                                          EXECUTABLES                                             #
####################################################################################################

# Service

add_executable(service
               src/service.cpp
               src/HaikuService.cpp)

target_link_libraries(service
                      ${PROJECT_NAME}_interfaces)

ament_target_dependencies(service
                          "rclcpp"
                          "std_msgs")

# Action Client

add_executable(client
               src/client.cpp
               src/HaikuClient.cpp)

target_link_libraries(client
                      ${PROJECT_NAME}_interfaces)

ament_target_dependencies(client
                          "rclcpp"
                          "std_msgs")

####################################################################################################
#                                         INSTALLATION                                             #
####################################################################################################

install(TARGETS
        service
        client
        DESTINATION lib/${PROJECT_NAME})
        
ament_package()
