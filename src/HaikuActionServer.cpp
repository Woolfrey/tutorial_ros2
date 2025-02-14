/**
 * @file   HaikuActionClient.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Source file for the HaikuActionServer class.
 */

#include <HaikuActionServer.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
HaikuActionServer::HaikuActionServer(const std::string &nodeName,
                                     const std::string &actionName)
                                     : Node(nodeName)
{
    using namespace std::placeholders;                                                              // _1, _2
    
    _actionServer = rclcpp_action::create_server<tutorial_ros2::action::Haiku>
    (
        this,                                                                                       // This node
        actionName,                                                                                 // Name to be advertised
        std::bind(&HaikuActionServer::handle_goal,     this, _1, _2),
        std::bind(&HaikuActionServer::handle_cancel,   this, _1),
        std::bind(&HaikuActionServer::handle_accepted, this, _1)
    );
    
    RCLCPP_INFO(this->get_logger(),
                "Started '%s` action server. Advertising '%s' action.",
                nodeName.c_str(), actionName.c_str());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Process an action request                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
rclcpp_action::GoalResponse
HaikuActionServer::handle_goal(const rclcpp_action::GoalUUID      &uuid,
                               std::shared_ptr<const Haiku::Goal> goal)
{
    (void)uuid;                                                                                     // This prevents colcon build from throwing a warning message
    
    RCLCPP_INFO(this->get_logger(), "Received action request.");
    
    if(goal->number_of_lines < 1)
    {
        RCLCPP_WARN(this->get_logger(), "Requested number of lines was less than 1.");
        
        return rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                     // Go immediately to handle_accepted() method
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      Prepare the action                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HaikuActionServer::handle_accepted(const std::shared_ptr<HaikuGoalHandle> goalHandle)
{
    std::thread{std::bind(&HaikuActionServer::execute, this, std::placeholders::_1), goalHandle}.detach();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     Implement the action                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HaikuActionServer::execute(const std::shared_ptr<HaikuGoalHandle> goalHandle)
{
    RCLCPP_INFO(this->get_logger(), "Reading you a haiku.");                                        // Inform user
    
    Haiku::Feedback::SharedPtr feedback = std::make_shared<Haiku::Feedback>();                      // We need this to store feedback
    
    Haiku::Result::SharedPtr result = std::make_shared<Haiku::Result>();                            // We need this to update the total result
    
    result->poem.data = "\n";                                                                       // Start from new line
    
    rclcpp::Rate loopRate(1);                                                                       // This is used to regulate the frequence of the for-loop
    
    int counter = 1;                                                                                // Used to keep track of which line in the haiku
    
    for(int i = 0; i < goalHandle->get_goal()->number_of_lines and rclcpp::ok(); ++i)
    {
        feedback->line_number = i+1;                                                                // Must be a positive integer
        
        switch(counter)
        {
            case 1:
            {
                feedback->current_line.data = "Worker bees can leave.\n";
                break;
            }
            case 2:
            {
                feedback->current_line.data = "Even drones can fly away.\n";
                break;
            }
            case 3:
            {
                feedback->current_line.data = "The Queen is their slave.\n";
                break;
            }
        }
        
        result->poem.data += feedback->current_line.data;                                           // Append to the result
        
        if(counter < 3) ++counter;
        else            counter = 1;
        
        // Check for cancellation request
        if(goalHandle->is_canceling())
        {
            goalHandle->canceled(result);
            
            RCLCPP_INFO(this->get_logger(), "Reading cancelled at line %d", i+1);
        }
        
        goalHandle->publish_feedback(feedback);
        
        loopRate.sleep();
    }

    // Finished
    if(rclcpp::ok())
    {
        goalHandle->succeed(result);                                                                // Add result

        RCLCPP_INFO(rclcpp::get_logger("haiku_action_server"), "Finished reading the haiku.");
    }
}
