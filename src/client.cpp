/**
 * @file   client.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  ROS2 executable that runs a client node.
 */
 
#include <HaikuClient.h>
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
    
    auto haikuClient = std::make_shared<HaikuClient>("haiku_client", "haiku_service");

    rclcpp::executors::SingleThreadedExecutor executor;                                             // We need this to run the node
    executor.add_node(haikuClient);                                                                 // Self explanatory
    executor.spin();                                                                                // Run the node indefinitely
    
    rclcpp::shutdown();                                                                             // As it says
    
    return 0;                                                                                       // No problems with main()
}
