/**
 * @file   HaikuSubscriber.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Source file for the HaikuSubscriber class.
 */
 
#include <HaikuSubscriber.h>
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
HaikuSubscriber::HaikuSubscriber(const std::string &nodeName,
                                 const std::string &topicName)
                                 : Node(nodeName)
{
    RCLCPP_INFO(this->get_logger(), "Waiting for the '%s' topic to be advertised...", topicName.c_str());
    
    int elapsedTime = 0;
    
    while (rclcpp::ok() && this->count_publishers(topicName) == 0)
    {
        if (elapsedTime >= 5000)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "The '%s' topic did not appear within 5 seconds of waiting. "
                         "Shutting down.", topicName.c_str());
                         
            rclcpp::shutdown();
            
            return;
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        elapsedTime += 500;
    }

    _subscriber = this->create_subscription<std_msgs::msg::String>(topicName,1, std::bind(&HaikuSubscriber::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Reading you a haiku:");
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Callback method                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HaikuSubscriber::callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "'%s'", msg->data.c_str());
}
