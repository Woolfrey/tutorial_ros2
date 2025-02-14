/**
 * @file   action_server.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Executable for running an HaikuActionServer.
 */

#include <HaikuActionServer.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                              MAIN                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);                                                                       // Start up ROS2
    
    auto haikuActionServer = std::make_shared<HaikuActionServer>("haiku_action_server", "haiku_action");
    
    rclcpp::executors::SingleThreadedExecutor executor;                                             // We need this to run the node
    executor.add_node(haikuActionServer);                                                           // Self explanatory
    executor.spin();                                                                                // Run the node indefinitely
    
    rclcpp::shutdown();                                                                             // As it says
    
    return 0;  
}
