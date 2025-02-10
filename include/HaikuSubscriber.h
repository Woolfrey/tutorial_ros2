/**
 * @file   HaikuSubscriber.h
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Header file for the HaikuSubscriber class.
 */

#ifndef HAIKU_SUBSCRIBER_H
#define HAIKU_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>                                                                        // Fundamental ROS2 C++ packages
#include <std_msgs/msg/string.hpp>                                                                  // String message type

/**
 * @brief A class that subscribes to and prints out a haiku.
 */
class HaikuSubscriber : public rclcpp::Node
{
    public:
        
        /**
         * @brief Constructor.
         * @param topicName The name for the message to subscribe to.
         * @param nodeName The name of the node that will appear on the ROS2 network.
         */
        HaikuSubscriber(const std::string &nodeName = "haiku_subscriber",
                        const std::string &topicName = "haiku");
        
    private:
    
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;                         ///< Object responsible for receiving messages over the ROS2 network
                
        /**
         * @brief This method is executed whenever a new message is published to the specified topic.
         * @param msg A placeholder for the message. This must match the template argument < > for the subscriber.
         */
        void callback(const std_msgs::msg::String::SharedPtr msg);
};                                                                                                  // Semicolon needed after a class declaration

#endif
