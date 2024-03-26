#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_control_msgs/srv/set_operating_mode.hpp"
#include "std_msgs/msg/int64.hpp"

class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  using SetOperatingMode = dynamixel_control_msgs::srv::SetOperatingMode;

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  void updateGoalCurrent();
  void publishPresentPosition();

  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<SetOperatingMode>::SharedPtr set_operating_mode_service_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr present_position_publisher_;

  int present_position;
  int goal_current;
};

#endif // READ_WRITE_NODE_HPP_
