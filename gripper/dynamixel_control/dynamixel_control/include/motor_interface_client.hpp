#ifndef MOTOR_INTERFACE_CLIENT_HPP_
#define MOTOR_INTERFACE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_control_msgs/srv/set_operating_mode.hpp"

class MotorInterfaceClient : public rclcpp::Node
{
public:
    explicit MotorInterfaceClient(const std::string & node_name, uint8_t operating_mode, uint32_t goal_current, uint32_t goal_position);

private:
    void sendSetOperatingModeRequests(uint8_t operating_mode, uint32_t goal_current, uint32_t goal_position);
    void sendSetOperatingModeRequest(uint8_t id, uint8_t operating_mode, uint32_t goal_current, uint32_t goal_position);

    rclcpp::Client<dynamixel_control_msgs::srv::SetOperatingMode>::SharedPtr set_operating_mode_client_;
};

#endif /* MOTOR_INTERFACE_CLIENT_HPP_ */
