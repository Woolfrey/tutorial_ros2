/**
 * @file   HaikuPublisher.h
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Header file for the HaikuPublisher class.
 */

#ifndef HAIKU_PUBLISHER_H
#define HAIKU_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>                                                                        // Fundamental ROS2 C++ packages
#include <std_msgs/msg/string.hpp>                                                                  // String message type

/**
 * @brief A class for continually publishing the lines of a Haiku.
 */
class HaikuPublisher : public rclcpp::Node
{
    public:
        
        /**
         * @brief Constructor.
         * @param topicName The name for the message that will be advertised.
         * @param nodeName The name of the node that will appear on the ROS2 network.
         * @param milliseconds The length of the delay before the next line is published.
         */
        HaikuPublisher(const std::string &nodeName = "haiku_publisher",
                       const std::string &topicName = "haiku",
                       const int &milliseconds = 2000);
    private:
    
        unsigned int _lineNumber = 1;                                                               ///< This is used to keep track of what line of the haiku is being published.

        rclcpp::TimerBase::SharedPtr _timer;                                                        ///< Object responsible for executing the callback method.
        
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;                             ///< Object responsible for sending message over the ROS2 network
                
        /**
         * @brief This method cycles through the lines of the haiku and publishes them one by one.
         */
        void
        timer_callback();
};                                                                                                  // Semicolon needed after a class declaration

#endif
