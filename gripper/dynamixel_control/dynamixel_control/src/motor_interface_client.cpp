#include "motor_interface_client.hpp"

MotorInterfaceClient::MotorInterfaceClient(const std::string & node_name, uint8_t operating_mode, uint32_t goal_current, uint32_t goal_position)
    : Node(node_name)
{
    set_operating_mode_client_ = this->create_client<dynamixel_control_msgs::srv::SetOperatingMode>("set_operating_mode");

    while (!set_operating_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Start asynchronous task to send service requests for both motors
    auto future = std::async(std::launch::async, &MotorInterfaceClient::sendSetOperatingModeRequests, this, operating_mode, goal_current, goal_position);

    // Wait for the future to complete
    future.wait();
}

void MotorInterfaceClient::sendSetOperatingModeRequests(uint8_t operating_mode, uint32_t goal_current, uint32_t goal_position)
{
    // Send service requests for both motors
    sendSetOperatingModeRequest(1, operating_mode, goal_current, goal_position);
}

void MotorInterfaceClient::sendSetOperatingModeRequest(uint8_t id, uint8_t operating_mode, uint32_t goal_current, uint32_t goal_position)
{
    auto request = std::make_shared<dynamixel_control_msgs::srv::SetOperatingMode::Request>();
    request->id = id;
    request->operating_mode = operating_mode;
    request->goal_current = goal_current;
    request->goal_position = goal_position;

    auto future = set_operating_mode_client_->async_send_request(request);

    try {
        // Wait for the response (blocking call)
        rclcpp::spin_until_future_complete(shared_from_this(), future);

        auto result = future.get();

        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Successfully set operating mode for motor %d", id);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode for motor %d", id);
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    uint8_t operating_mode = std::stoi(argv[1]);
    uint32_t goal_current = std::stoi(argv[2]);
    uint32_t goal_position = std::stoi(argv[3]);
    auto node = std::make_shared<MotorInterfaceClient>("motor_interface_client", operating_mode, goal_current, goal_position);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
