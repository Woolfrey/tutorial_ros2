/**
 * @file   subscriber.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  ROS2 executable that runs a node which publishes a haiku.
 */
 
#include "HaikuSubscriber.h"
#include "rclcpp/rclcpp.hpp"
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
    
    auto haikuSubscriber = std::make_shared<HaikuSubscriber>("haiku_subscriber", "haiku");          // Create an instance of the class / node

    rclcpp::executors::SingleThreadedExecutor executor;                                             // We need this to run the node
    executor.add_node(haikuSubscriber);                                                             // Self explanatory
    executor.spin();                                                                                // Run the node indefinitely
    
    rclcpp::shutdown();                                                                             // As it says
    
    return 0;                                                                                       // No problems with main()
}
