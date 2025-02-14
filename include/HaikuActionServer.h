/**
 * @file   HaikuActionServer.h
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Header file for the HaikuActionServer class.
 */

#ifndef HAIKU_ACTION_SERVER_H
#define HAIKU_ACTION_SERVER_H

#include <rclcpp/rclcpp.hpp>                                                                        // Fundamental ROS2 C++ packages
#include "rclcpp_action/rclcpp_action.hpp"                                                          // ROS2 C++ action library
#include <std_msgs/msg/string.hpp>                                                                  // String message type
#include "tutorial_ros2/action/haiku.hpp"                                                           // Custom action definition

/**
 * @brief A class for the Haiku action.
 */
class HaikuActionServer : public rclcpp::Node
{
    using Haiku = tutorial_ros2::action::Haiku;                                                     // Makes code shorter 
    using HaikuGoalHandle = rclcpp_action::ServerGoalHandle<Haiku>;                                 // Makes code shorter
    
    public:
        
        /**
         * @brief Constructor.
         * @param nodeName The name of the node that will be seen over ROS2.
         * @param actionName The name that will be advertised over ROS2.
         */
        HaikuActionServer(const std::string &nodeName    = "haiku_action_server",
                          const std::string &serviceName = "haiku_action");
                          
    private:
    
        rclcpp_action::Server<Haiku>::SharedPtr _actionServer;                                      ///< This handles action requests
         
        /**
         * @param Process a request from a client.
         * @param uuid A unique ID for this particular request.
         * @param goal The goal portion of the Haiku.action
         * @return 
         */
        rclcpp_action::GoalResponse
        handle_goal(const rclcpp_action::GoalUUID      &uuid,
                    std::shared_ptr<const Haiku::Goal> goal);
        
        /**
         * @brief Prepares the action for execution.
         * @param goalHandle Used to manage the action.
         */
        void
        handle_accepted(const std::shared_ptr<HaikuGoalHandle> goalHandle);
        
        /**
         * @brief Reads out lines of a haiku one-at-a-time.
         * @param goalHandle Manages the execution of the action.
         */
        void
        execute(const std::shared_ptr<HaikuGoalHandle> goalHandle);
        
        /**
         * @brief Processes a cancel request.
         * @return rclcpp_action::CancelResponse::ACCEPT or REJECT
         */
        rclcpp_action::CancelResponse
        handle_cancel(const std::shared_ptr<HaikuGoalHandle> goalHandle)
        {
             (void)goalHandle;                                                                      // Stops colcon build throwing a warning
             
             RCLCPP_INFO(rclcpp::get_logger("haiku_action_server"),"Received cancellation request."); // Inform user
            
             return rclcpp_action::CancelResponse::ACCEPT;
        }
        
};                                                                                                  // Semicolon needed after a class declaration.
        
#endif
