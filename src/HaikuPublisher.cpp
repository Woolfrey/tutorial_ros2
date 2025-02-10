/**
 * @file   HaikuPublisher.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Source files for the HaikuPublisher class.
 */
 
#include <HaikuPublisher.h>
#include <functional>
#include <chrono>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
HaikuPublisher::HaikuPublisher(const std::string &nodeName,
                               const std::string &topicName,
                               const int &milliseconds)
                               : Node(nodeName)
{
    _publisher = this->create_publisher<std_msgs::msg::String>(topicName, 1);                       // Create the publisher, advertise the given topic name.

    _timer = this->create_wall_timer(
        std::chrono::milliseconds(milliseconds),                                                    // Convert to milliseconds
        std::bind(&HaikuPublisher::timer_callback, this)                                            // Attach callback function, this node
    );
    
    RCLCPP_INFO(this->get_logger(),
                "Created the '%s' node. Publishing to the '%s' topic name.",
                nodeName.c_str(), topicName.c_str());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Callback method                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HaikuPublisher::timer_callback()
{
    std_msgs::msg::String message;                                                                  // This is what we want to publish.

    if     (_lineNumber == 1) message.data = "Worker bees can leave.";
    else if(_lineNumber == 2) message.data = "Even drones can fly away.";
    else if(_lineNumber == 3) message.data = "The Queen is their slave.";
    
    RCLCPP_INFO(this->get_logger(), "Publishing line number %d.", _lineNumber);                     // Inform the user

    if(_lineNumber < 3) _lineNumber++;                                                              // Go to next line of poem
    else                _lineNumber = 1;                                                            // Back to first

    _publisher->publish(message);                                                                   // Send message over ROS2 network
}
