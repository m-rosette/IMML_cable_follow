#ifndef DUAL_MOTOR_INTERFACE_HPP_
#define DUAL_MOTOR_INTERFACE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_control_msgs/srv/dual_set_operating_mode.hpp"

class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  using DualSetOperatingMode = dynamixel_control_msgs::srv::DualSetOperatingMode;

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  void updateGoalCurrent();

  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<DualSetOperatingMode>::SharedPtr set_operating_mode_service_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

  int present_position;
  int goal_current;
};

#endif  // DUAL_MOTOR_INTERFACE_HPP_
