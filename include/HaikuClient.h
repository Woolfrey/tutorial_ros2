/**
 * @file   HaikuClient.h
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Header file for the HaikuClient class.
 */

#ifndef HAIKU_CLIENT_H
#define HAIKU_CLIENT_H

#include <rclcpp/rclcpp.hpp>                                                                        // Fundamental ROS2 C++ packages
#include <std_msgs/msg/string.hpp>                                                                  // String message type
#include "tutorial_ros2/srv/haiku.hpp"                                                              // Custom service type

/**
 * @brief A class for sending requests to the Haiku service.
 */
class HaikuClient : public rclcpp::Node
{
    using Haiku = tutorial_ros2::srv::Haiku;                                                        // Makes code a little neater
    
    public:
        
        /**
         * @brief Constructor.
         * @param nodeName The name for this node which will be seen on the ROS2 network.
         * @param serviceName The name of the service being advertised by the server.
         */
        HaikuClient(const std::string &nodeName    = "haiku_client",
                    const std::string &serviceName = "haiku_service");
    
    private:
    
        rclcpp::Client<Haiku>::SharedPtr _client;                                                   ///< This is responsible for sending requests to the server
        
        /**
         * @brief Allows user to enter a command via the console.
         */
        void
        get_user_input();
        
        /**
         * @brief Embeds line number in appropriate message type and sends to server.
         * @param lineNumber Which line of the haiku is requested from the server.
         */
        void send_request(const int &lineNumber);
        
        /**
         * @brief Prints out the response from the server.
         */
        void process_response(rclcpp::Client<tutorial_ros2::srv::Haiku>::SharedFuture future);
    
};                                                                                                  // Semicolon needed after class declaration

#endif
