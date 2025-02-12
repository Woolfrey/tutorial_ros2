/**
 * @file   HaikuService.h
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Header file for the HaikuService class.
 */

#ifndef HAIKU_SERVICE_H
#define HAIKU_SERVICE_H

#include <rclcpp/rclcpp.hpp>                                                                        // Fundamental ROS2 C++ packages
#include <std_msgs/msg/string.hpp>                                                                  // String message type
#include "tutorial_ros2/srv/haiku.hpp"                                                              // Custom service type

/**
 * @brief A class for sending a single line of a Haiku upon request.
 */
class HaikuService : public rclcpp::Node
{
    using Haiku = tutorial_ros2::srv::Haiku;                                                        // Makes code easier to read
    
    public:
        
        /**
         * @brief Constructor.
         * @param nodeName The name for the message that will be advertised.
         * @param serviceName The name of the node that will appear on the ROS2 network.
         */
        HaikuService(const std::string &nodeName    = "haiku_service",
                     const std::string &serviceName = "haiku_service");
                     
    private:
    
        rclcpp::Service<Haiku>::SharedPtr _service;                                                 ///< This object is responsible for receiving service requests
        
        /**
         * @brief Retrieves the requested line of a haiku.
         */
        void
        get_line(const std::shared_ptr<Haiku::Request>  request,
                       std::shared_ptr<Haiku::Response> response);
};                                                                                                  // Semicolon needed after a class declaration

#endif
