#include "dual_motor_client.hpp"

DualMotorClient::DualMotorClient(uint8_t operating_mode, uint32_t operation_target)
  : Node("dual_motor_client"),
    operating_mode_(operating_mode),
    operation_target1_(operation_target),
    operation_target2_(operation_target)
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

  // Set bounds for operation_target
  uint32_t upper_pos_bound = 500;
  uint32_t lower_pos_bound = 50;
  uint32_t upper_current_bound = 100;
  uint32_t lower_current_bound = 10;

  // Check bounds on operation_target (position)
  if (operating_mode_ == 3 && operation_target1_ > upper_pos_bound) {
    RCLCPP_WARN(this->get_logger(), "Position upper bound exceeded - capping value. [Goal Position: %d]", upper_pos_bound);
    operation_target1_ = upper_pos_bound;
  }
  else if (operating_mode_ == 3 && operation_target1_ < lower_pos_bound) { 
    RCLCPP_WARN(this->get_logger(), "Position lower bound exceeded - capping value. [Goal Position: %d]", lower_pos_bound);
    operation_target1_ = lower_pos_bound;
  }

  // Check bounds on operation_target (current)
  if (operating_mode_ == 0 && operation_target1_ > upper_current_bound) {
    RCLCPP_WARN(this->get_logger(), "Current upper bound exceeded - capping value. [Goal Current: %d]", upper_current_bound);
    operation_target1_ = upper_current_bound;
  }
  else if (operating_mode_ == 0 && operation_target1_ < lower_current_bound) { 
    RCLCPP_WARN(this->get_logger(), "Current lower bound exceeded - capping value. [Goal Current: %d]", lower_current_bound);
    operation_target1_ = lower_current_bound;
  }

  request->operation_target1 = operation_target1_;

  // Calculate the modified (inverted) operation target for the second motor
  uint32_t modified_operation_target = operation_target1_;
  if (request->id2 == 2 && operating_mode_ == 3) { // If it's the second motor and in position mode
      modified_operation_target = 4075 - operation_target1_; // Calculate the modified target position
  }
  else if (request->id2 == 2 && operating_mode_ == 0) { // If it's the second motor and in position mode
      modified_operation_target = - operation_target1_; // Make the operation_target negative
  }

  request->operation_target2 = modified_operation_target;

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
