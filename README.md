# 1. Publishers & Subscribers

[🔙 Back to `main`](https://github.com/Woolfrey/tutorial_ros2/blob/main/README.md#ros2-c-tutorials)

### Contents:
- [1.1 Creating a Publisher](https://github.com/Woolfrey/tutorial_ros2/blob/publisher/README.md#11-creating-a-publisher)
- [1.2 Creating a Subscriber](https://github.com/Woolfrey/tutorial_ros2/blob/publisher/README.md#12-creating-a-subscriber)


The `Publisher` and `Subcriber` protocol is analogous to the role of a news agency, or book store. A printing press will publish magazines and/or books that are sent to a store. They are made publically available for people to purchase of their own volition. The type of data being communicated is fast, frequent, and numerous.

<p align="center">
  <img src="assets/PublisherSubscriberModel.png" width="400" height="auto" alt="Publisher/Subscriber Model."/>
  <br>
  <em>Figure 1: Publishers make data publicly available for any number of subscribers.</em>
</p>

Folder structure :file_folder:
```
ros2_workspace/src/
├─ include/
    ├─  HaikuPublisher.h
    └── HaikuSubscriber.h
├─ src/
    ├─  HaikuPublisher.cpp
    ├─  HaikuSubscriber.cpp
    ├─  publisher.cpp
    └── subscriber.cpp
├── CMakeLists.txt
└── package.xml
```

## 1.1 Creating a Publisher

### 1.1.1 Create the Header File :page_facing_up:

It is good practice in C++ to separate the declarations for functions and classes from the source code. This makes it more efficient for the computer to compile large projects.

ROS2 is also designed around the use of object-oriented programming (OOP). This is useful because we can:
1. Spawn multiple objects of the same type (e.g. multiple cameras in a single robot system), and
2. Launch multiple nodes within a single executable.

Inside of `include/HaikuPublisher.h` insert the following code:

```
#ifndef HAIKU_PUBLISHER_H
#define HAIKU_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class HaikuPublisher : public rclcpp::Node
{
    public:

        HaikuPublisher(const std::string &nodeName = "haiku_publisher",
                       const std::string &topicName = "haiku",
                       const int &milliseconds = 2000);
    private:
    
        unsigned int _lineNumber = 1;

        rclcpp::TimerBase::SharedPtr _timer;
        
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
                
        void timer_callback();
};

#endif
```

The important lines of code to consider here are:

- `#include <rclcpp/rclcpp.hpp>`: The ROS2 C++ client library which provides all the necessary functionality.
- `class HaikuPublisher : public rclcpp::Node`: Or HaikuPublisher class inherits the ROS2 Node class, and all its functions (methods) and variables (members).
- `rclcpp::TimerBase::SharedPtr _timer`: This is used to regulate how often we publish messages.
- `rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher`: This object is directly responsible for publishing our message over the ROS2 network.
- `void timer_callback();` This will be combined with the `_timer` and perform the core work for this class.

### 1.1.2 Create the Source File :page_facing_up:

```
#include <HaikuPublisher.h>
#include <functional>
#include <chrono>

HaikuPublisher::HaikuPublisher(const std::string &nodeName,
                               const std::string &topicName,
                               const int &milliseconds)
                               : Node(nodeName)
{
    _publisher = this->create_publisher<std_msgs::msg::String>(topicName, 1);

    _timer = this->create_wall_timer(
        std::chrono::milliseconds(milliseconds),
        std::bind(&HaikuPublisher::timer_callback, this)
    );
    
    RCLCPP_INFO(this->get_logger(),
                "Created the '%s' node. Publishing to the '%s' topic name.",
                nodeName.c_str(), topicName.c_str());
}

void HaikuPublisher::timer_callback()
{
    std_msgs::msg::String message;

    if     (_lineNumber == 1) message.data = "Worker bees can leave.";
    else if(_lineNumber == 2) message.data = "Even drones can fly away.";
    else if(_lineNumber == 3) message.data = "The Queen is their slave.";
    
    RCLCPP_INFO(this->get_logger(), "Publishing line number %d.", _lineNumber);

    if(_lineNumber < 3) _lineNumber++;
    else                _lineNumber = 1;

    _publisher->publish(message);
}
```

```
_publisher = this->create_publisher<std_msgs::msg::String>(topicName, 1);
```
- `this->` refers to the `rclcpp::Node`, and is attaching the publisher object to it.
- The `<std_msgs::msg::String>` is a template argument, using a ROS string message type.
- The `topicName` argument is what will appear to other nodes on the ROS2 network.
- The `1` argument refers to the queue length for number of messages available at a time.

Any subscribers to this topic will:
1. Find and connect to the topic via a matching `topicName`, and
2. Require the same type of message template, in this case the `std_msgs::msg::String`.


### 1.1.3 Create the Executable :gear:

### 1.1.4 Create Configuration Files :hammer_and_wrench:

### 1.1.5 Compiling & Running the Package :computer:

### 1.1.6 The Code Explained 🔎

[⬆️ Return to top.](https://github.com/Woolfrey/tutorial_ros2/blob/publisher/README.md#1-publishers--subscribers)

## 1.2 Creating a Subscriber

### 1.2.1 Create the Header File :page_facing_up:

### 1.2.2 Create the Source File :page_facing_up:

### 1.2.3 Create the Executable :gear:

### 1.2.4 Create Configuration Files :hammer_and_wrench:

### 1.2.5 Compiling & Running the Package :computer:

### 1.2.6 The Code Explained 🔎

[⬆️ Return to top.](https://github.com/Woolfrey/tutorial_ros2/blob/publisher/README.md#1-publishers--subscribers)

[🔙 Back to `main`](https://github.com/Woolfrey/tutorial_ros2/blob/main/README.md#ros2-c-tutorials)
