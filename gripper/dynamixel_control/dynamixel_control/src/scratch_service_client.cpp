#include "scratch_service_client.hpp"

DualMotorClient::DualMotorClient(uint8_t operating_mode, uint32_t operation_target)
  : Node("dual_motor_client"),
    operating_mode_(operating_mode),
    operation_target_(operation_target)
{
  // Create service client
  set_operating_mode_client_ = this->create_client<dynamixel_control_msgs::srv::DualSetOperatingMode>("set_operating_mode");

  // Wait for the service to be available
  while (!set_operating_mode_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  // Create request
  auto request = std::make_shared<dynamixel_control_msgs::srv::DualSetOperatingMode::Request>();
  request->id1 = 1; // Assuming motor IDs are 1 and 2
  request->id2 = 2;
  request->operating_mode = operating_mode_;
  request->operation_target1 = operation_target_;

  // Send request
  auto future = set_operating_mode_client_->async_send_request(request, std::bind(&DualMotorClient::responseCallback, this, std::placeholders::_1));
}

void DualMotorClient::responseCallback(rclcpp::Client<dynamixel_control_msgs::srv::DualSetOperatingMode>::SharedFuture future)
{
  try {
    auto result = future.get();
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully set operating mode for both motors");
      rclcpp::shutdown();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode for both motors");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: dual_motor_client <operating_mode> <operation_target>");
    return 1;
  }

  uint8_t operating_mode = std::atoi(argv[1]);
  uint32_t operation_target = std::atoi(argv[2]);

  // Create a shared pointer to manage the lifetime of the node object
  auto node = std::make_shared<DualMotorClient>(operating_mode, operation_target);

  // Spin the node
  rclcpp::spin(node);

  return 0;
}
