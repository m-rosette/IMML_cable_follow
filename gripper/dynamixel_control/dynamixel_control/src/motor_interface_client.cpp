#include "motor_interface_client.hpp"

MotorInterfaceClient::MotorInterfaceClient(uint8_t operating_mode, uint32_t goal_current, uint32_t goal_position)
    : Node("motor_client"),
      operating_mode_(operating_mode),
      goal_current_(goal_current),
      goal_position_(goal_position)
{
    // Create service client
    set_operating_mode_client_ = this->create_client<dynamixel_control_msgs::srv::SetOperatingMode>("set_operating_mode");

    // Wait for the service to be available
    while (!set_operating_mode_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Create request
    auto request = std::make_shared<dynamixel_control_msgs::srv::SetOperatingMode::Request>();
    request->id = 1;
    request->operating_mode = operating_mode_;

    // ---------------- CHANGE AS NEEDED ----------------
    // Set bounds for operation_target
    uint32_t upper_pos_bound = 500;
    uint32_t lower_pos_bound = 50;
    uint32_t upper_current_bound = 10;
    uint32_t lower_current_bound = 0.5;

    // Check bounds on position
    if (goal_position_ > upper_pos_bound)
    {
        RCLCPP_WARN(this->get_logger(), "Position upper bound exceeded - capping value. [Goal Position: %d]", upper_pos_bound);
        goal_position_ = upper_pos_bound;
    }
    else if (goal_position_ < lower_pos_bound)
    {
        RCLCPP_WARN(this->get_logger(), "Position lower bound exceeded - capping value. [Goal Position: %d]", lower_pos_bound);
        goal_position_ = lower_pos_bound;
    }

    // Check bounds on current
    if (goal_current_ > upper_current_bound)
    {
        RCLCPP_WARN(this->get_logger(), "Current upper bound exceeded - capping value. [Goal Current: %d]", upper_current_bound);
        goal_current_ = upper_current_bound;
    }
    else if (goal_current_ < lower_current_bound)
    {
        RCLCPP_WARN(this->get_logger(), "Current lower bound exceeded - capping value. [Goal Current: %d]", lower_current_bound);
        goal_current_ = lower_current_bound;
    }

    request->goal_current = goal_current_;
    request->goal_position = goal_position_;

    // Send request
    auto future = set_operating_mode_client_->async_send_request(request, std::bind(&MotorInterfaceClient::responseCallback, this, std::placeholders::_1));
}

void MotorInterfaceClient::responseCallback(rclcpp::Client<dynamixel_control_msgs::srv::SetOperatingMode>::SharedFuture future)
{
    try
    {
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully set operating mode for both motors");
            rclcpp::shutdown();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode for both motors");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (argc != 4)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: motor_interface_client <operating_mode> <goal_current> <goal_position>");
        return 1;
    }
    uint8_t operating_mode = std::stoi(argv[1]);
    uint32_t goal_current = std::stoi(argv[2]);
    uint32_t goal_position = std::stoi(argv[3]);

    // Create a shared pointer to manage the lifetime of the node object
    auto node = std::make_shared<MotorInterfaceClient>(operating_mode, goal_current, goal_position);
    // Spin the node
    rclcpp::spin(node);

    // Shutdown cleanly
    rclcpp::shutdown();

    return 0;
}
