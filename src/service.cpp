/**
 * @file   server.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  ROS2 executable that runs a service node.
 */
 
#include "HaikuService.h"
#include "rclcpp/rclcpp.hpp"
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
    
    auto haikuService = std::make_shared<HaikuService>("haiku_service", "haiku_service");

    rclcpp::executors::SingleThreadedExecutor executor;                                             // We need this to run the node
    executor.add_node(haikuService);                                                                // Self explanatory
    executor.spin();                                                                                // Run the node indefinitely
    
    rclcpp::shutdown();                                                                             // As it says
    
    return 0;                                                                                       // No problems with main()
}
