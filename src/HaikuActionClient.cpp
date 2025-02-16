/**
 * @file   HaikuActionClient.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Source file for the HaikuActionClient class.
 */

#include <HaikuActionClient.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
HaikuActionClient::HaikuActionClient(const std::string &nodeName,
                                     const std::string &actionName)
                                     : Node(nodeName)
{
    _actionClient = rclcpp_action::create_client<Haiku>(this,actionName);                           // Create & connect client to advertised action
    
    // Wait for the action to be advertised before proceeding
    
    RCLCPP_INFO(this->get_logger(), "Waiting for `%s` action to be advertised...", actionName.c_str());
    
    if(not _actionClient->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "Waited 5 seconds and found nothing. Shutting down");
        
        rclcpp::shutdown();
    }
    
    // Set the callback methods
    
    using namespace std::placeholders;
    
    _sendGoalOptions = rclcpp_action::Client<Haiku>::SendGoalOptions();
    _sendGoalOptions.result_callback = std::bind(&HaikuActionClient::result_callback, this, _1);
    _sendGoalOptions.feedback_callback = std::bind(&HaikuActionClient::feedback_callback, this, _1, _2);
    _sendGoalOptions.goal_response_callback = std::bind(&HaikuActionClient::goal_response_callback, this, _1);
       
       
    std::thread(&HaikuActionClient::get_user_input, this).detach();                                 // Run this method in a separate thread
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Process input from the terminal                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HaikuActionClient::get_user_input()
{
    RCLCPP_INFO(this->get_logger(), "Please enter a number > 0, or hit Enter to cancel.");          // Inform user
    
    while(rclcpp::ok())
    {
        std::string userInput;                                                                      // Storage location
        
        std::getline(std::cin, userInput);                                                          // Registes Enter as an input
        
        uint8_t status = (_activeGoalHandle == nullptr) ? 0 : _activeGoalHandle->get_status();      // As it says

        if (userInput.empty())                                                                      // Empty, i.e. cancel request
        {   
            if (_activeGoalHandle == nullptr
            or  status == GoalStatus::STATUS_ABORTED
            or  status == GoalStatus::STATUS_CANCELED
            or  status == GoalStatus::STATUS_SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "No action currently running.");
            }
            else if (status == GoalStatus::STATUS_CANCELING)
            {
                RCLCPP_INFO(this->get_logger(), "Action canceling. Please be patient.");
            }
            else // ACCEPTED, EXECUTING, UNKNOWN
            {
                _actionClient->async_cancel_goal(_activeGoalHandle);
            }
        }
        else
        {
            int numberOfLines;
            
            // If input is not an integer, std::stoi will throw a runtime error
            // so we need to catch it here.
            try
            {
                numberOfLines = std::stoi(userInput);
            }
            catch(const std::exception &exception)
            {
                RCLCPP_WARN(this->get_logger(), "Invalid argument.");
                
                continue;
            }
               
            if (numberOfLines > 0)
            {
                if (status == GoalStatus::STATUS_ABORTED
                or  status == GoalStatus::STATUS_CANCELED
                or  status == GoalStatus::STATUS_SUCCEEDED
                or  status == GoalStatus::STATUS_UNKNOWN)
                {
                    auto goal = Haiku::Goal();                                                      // Get the goal component of the Haiku.action
                    
                    goal.number_of_lines = numberOfLines;                                           // Insert the number of lines
                    
                    _actionClient->async_send_goal(goal, _sendGoalOptions);                         // Send to the client
                }
                else if(status == GoalStatus::STATUS_ACCEPTED
                     or status == GoalStatus::STATUS_EXECUTING)
                {
                    RCLCPP_WARN(this->get_logger(), "Another action is currently running.");
                }
                else                                                                                // CANCELING
                {
                    RCLCPP_INFO(this->get_logger(), "Previous action currently canceling. Please wait.");
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Argument must be a positive integer.");
            }
        }
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Process the response to a request                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HaikuActionClient::goal_response_callback(std::shared_ptr<HaikuGoalHandle> goalHandle)
{
    if(not goalHandle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
    }
    else
    {
        _activeGoalHandle = goalHandle;
        
        RCLCPP_INFO (this->get_logger(), "Goal was accepted by server. Waiting for the result.");
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Listen to feedback topic                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HaikuActionClient::feedback_callback(std::shared_ptr<HaikuGoalHandle> goalHandle,
                                     std::shared_ptr<const Haiku::Feedback> feedback)
{
    (void)goalHandle;
    
    RCLCPP_INFO(this->get_logger(), "The current line number is %d: %s", feedback->line_number, feedback->current_line.data.c_str());
}
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Process the result of the action                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HaikuActionClient::result_callback(const HaikuGoalHandle::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(this->get_logger(), "Here is the result:\n%s", result.result->poem.data.c_str());
            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_ERROR(this->get_logger(), "Action was aborted.");
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            RCLCPP_ERROR(this->get_logger(), "Action was canceled.");
            break;
        }
        default:
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown result code (how did that happen?).");
            break;
        }
    }
    
    _activeGoalHandle = nullptr;
    
    RCLCPP_INFO(this->get_logger(), "Please enter a number > 0, or hit Enter to cancel.");
}                    

