#ifndef MOTOR_INTERFACE_CLIENT_HPP_
#define MOTOR_INTERFACE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_control_msgs/srv/set_operating_mode.hpp"

class MotorInterfaceClient : public rclcpp::Node
{
public:
    MotorInterfaceClient(uint8_t operating_mode, uint32_t goal_current, uint32_t goal_position);

private:
    void responseCallback(rclcpp::Client<dynamixel_control_msgs::srv::SetOperatingMode>::SharedFuture future);

    rclcpp::Client<dynamixel_control_msgs::srv::SetOperatingMode>::SharedPtr set_operating_mode_client_;
    uint8_t operating_mode_;
    uint32_t goal_current_;
    uint32_t goal_position_;
};

#endif /* MOTOR_INTERFACE_CLIENT_HPP_ */
