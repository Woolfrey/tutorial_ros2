/**
 * @file   HaikuService.cpp
 * @author Jon Woolfrey
 * @data   February 2025
 * @brief  Source code for the HaikuService class.
 */

#include <HaikuService.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
HaikuService::HaikuService(const std::string &nodeName,
                           const std::string &serviceName)
                           : Node(nodeName)
{
    using namespace std::placeholders;                                                              // std::placeholders_1, _2
    
    _service = this->create_service<Haiku>(serviceName, std::bind(&HaikuService::get_line, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Advertising service called '%s'.", serviceName.c_str());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Process service request                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HaikuService::get_line(const std::shared_ptr<Haiku::Request>  request,
                             std::shared_ptr<Haiku::Response> response)
{
     switch(request->line_number)
     {    
          case 1:
          {
               response->line.data = "Worker bees can leave.";
               break;
          }
          case 2:
          {
               response->line.data = "Even drones can fly away.";
               break;
          }
          case 3:
          {
               response->line.data = "The Queen is their slave.";
               break;
          }
          default:
          {
               response->line.data = "FLAGRANT REQUEST ERROR: Expected request of 1, 2, or 3 but yours was "
                             + std::to_string(request->line_number);
               break;
          }
     }
}
