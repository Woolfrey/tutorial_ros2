/**
 * @file   action_client.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Executable for running an HaikuActionClient.
 */

#include <HaikuActionClient.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                              MAIN                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);                                                                       // Start up ROS2
    
    auto haikuActionClient = std::make_shared<HaikuActionClient>("haiku_action_client", "haiku_action");
    
    rclcpp::executors::SingleThreadedExecutor executor;                                             // We need this to run the node
    executor.add_node(haikuActionClient);                                                           // Self explanatory
    executor.spin();                                                                                // Run the node indefinitely
    
    rclcpp::shutdown();                                                                             // As it says
    
    return 0;  
}
