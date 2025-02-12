/**
 * @file   HaikuClient.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Source code for the HaikuClient class.
 */

#include <HaikuClient.h>
#include <thread>                                                                                   // Needed to create threads (duh!)

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
HaikuClient::HaikuClient(const std::string &nodeName,
                         const std::string &serviceName)
                         : Node(nodeName)
{
    _client = this->create_client<tutorial_ros2::srv::Haiku>(serviceName);                          // Create client for advertised service

    RCLCPP_INFO(this->get_logger(), "Waiting for `%s` service to be advertised...", serviceName.c_str());

    auto startTime = std::chrono::steady_clock::now();

    while (not _client->wait_for_service(std::chrono::milliseconds(500)))
    {
        if (not rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service. Exiting...");
            
            return;
        }

        if (std::chrono::steady_clock::now() - startTime > std::chrono::seconds(5))
        {
            RCLCPP_ERROR(this->get_logger(), "Service `%s` not available after 5 seconds. Shutting down...", serviceName.c_str());
            
            rclcpp::shutdown();
            
            return;
        }
    }  
    
    std::thread(&HaikuClient::get_user_input, this).detach();                                       // Run the method in a separate thread
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Process service request                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
void HaikuClient::get_user_input()
{
    int userInput;

    while (rclcpp::ok())                                                                            // Loop until ROS is shut down
    {
        RCLCPP_INFO(this->get_logger(), "Please enter a number (1, 2, or 3):");                     // Inform user
        
        std::cin >> userInput;                                                                      // Get input from user

        if (std::cin.fail()) 
        {
            std::cin.clear();                                                                       // Clear input
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            RCLCPP_WARN(this->get_logger(), "Invalid input.");                                      // Inform user
            continue;
        }

        if (userInput == 1 or userInput == 2 or userInput == 3)
        {
            send_request(userInput);                                                                // Go to this method below
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Number must be between 1 and 3.");
        }
    }
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Assign input to message field, send to server                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void HaikuClient::send_request(const int &lineNumber)
{
    /* Haiku.srv:
     * # Goal
     * int64 line_number
     * ---
     * std_msgs/String line
     */
    
    auto request = std::make_shared<tutorial_ros2::srv::Haiku::Request>();                          // Temporary object
    
    request->line_number = lineNumber;                                                              // Add to field

    auto future = _client->async_send_request(request,
                                              std::bind(&HaikuClient::process_response, this, std::placeholders::_1)); // Bind method below
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Print out the line of the Haiku                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void HaikuClient::process_response(rclcpp::Client<tutorial_ros2::srv::Haiku>::SharedFuture future)
{
    auto response = future.get();

    if (response)
    {
        RCLCPP_INFO(this->get_logger(), "%s", response->line.data.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service request failed.");
    }
}
