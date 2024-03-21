#ifndef DUAL_MOTOR_CLIENT_HPP_
#define DUAL_MOTOR_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_control_msgs/srv/dual_set_operating_mode.hpp"

class DualMotorClient : public rclcpp::Node
{
public:
  DualMotorClient(uint8_t operating_mode, uint32_t operation_target);

private:
  void responseCallback(rclcpp::Client<dynamixel_control_msgs::srv::DualSetOperatingMode>::SharedFuture future);

  rclcpp::Client<dynamixel_control_msgs::srv::DualSetOperatingMode>::SharedPtr set_operating_mode_client_;
  uint8_t operating_mode_;
  uint32_t operation_target1_;
  uint32_t operation_target2_;
};

#endif  // DUAL_MOTOR_CLIENT_HPP_
