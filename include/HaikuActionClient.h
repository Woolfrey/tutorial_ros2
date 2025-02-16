/**
 * @file   HaikuActionClient.h
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Header file for the HaikuActionClient class.
 */

#ifndef HAIKU_ACTION_CLIENT_H
#define HAIKU_ACTION_CLIENT_H

#include <rclcpp/rclcpp.hpp>                                                                        // Fundamental ROS2 C++ packages
#include "rclcpp_action/rclcpp_action.hpp"                                                          // ROS2 C++ action library
#include <std_msgs/msg/string.hpp>                                                                  // String message type
#include "tutorial_ros2/action/haiku.hpp"                                                           // Custom action definition

/**
 * @brief A class for the Haiku action client.
 */
class HaikuActionClient : public rclcpp::Node
{
    using GoalStatus = action_msgs::msg::GoalStatus;
    using Haiku = tutorial_ros2::action::Haiku;                                                     // Makes code shorter 
    using HaikuGoalHandle = rclcpp_action::ClientGoalHandle<Haiku>;                                 // Makes code shorter
    
    public:
        
        /**
         * @brief Constructor.
         * @param nodeName The name of the node that will be seen over ROS2.
         * @param actionName The name that will be advertised over ROS2.
         */
        HaikuActionClient(const std::string &nodeName   = "haiku_action_client",
                          const std::string &actionName = "haiku_action");

    private:

        rclcpp_action::Client<Haiku>::SendGoalOptions _sendGoalOptions;                             ///< Use this to bind callback methods
            
        rclcpp_action::Client<Haiku>::SharedPtr _actionClient;                                      ///< This handles communication with the server
        
        std::shared_ptr<HaikuGoalHandle> _activeGoalHandle;                                         ///< Used to keep track
        
        /**
         * @brief Processes input from the terminal.
         */
        void
        get_user_input();
        
        /**
         * @brief Process the response from the server after a goal is sent.
         * @param future Future goal handle that will be returned by the server.
         */
        void
        goal_response_callback(std::shared_ptr<HaikuGoalHandle> goalHandle);

        /**
         * @brief Listens to the feedback topic.
         * @param HaikuGoalHandle::SharedPtr A pointer to the goal handle
         * @param feedback Pointer to the feedback portion of the action
         */
        void feedback_callback(std::shared_ptr<HaikuGoalHandle> goalHandle,
                               std::shared_ptr<const Haiku::Feedback> feedback);
                        
        /**
         * @brief Processes the result sent by the action server.
         * @param result Data structure with info on action result
         */
        void
        result_callback(const HaikuGoalHandle::WrappedResult &result);     
};                                                                                                  // Semicolon needed after a class declaration.
        
#endif
