// Code modified from ROBOTIS dynamixel_sdk_examples read_write_node.cpp

#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "rcutils/cmdline_parser.h"
#include "motor_interface.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_GOAL_CURRENT 102

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // Initialize the present position publisher
  present_position_publisher_ = this->create_publisher<std_msgs::msg::Int64>("present_position", 10);

  set_position_subscriber_ =
    this->create_subscription<SetPosition>(
    "set_position",
    QOS_RKL10V,
    [this](const SetPosition::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      uint32_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32

      // Write Goal Position (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id,
        ADDR_GOAL_POSITION,
        goal_position,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
      }
    }
    );

  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);

  auto set_operating_mode_callback =
    [this](
    const std::shared_ptr<SetOperatingMode::Request> request,
    std::shared_ptr<SetOperatingMode::Response> response) -> void
    {
      uint8_t dxl_error = 0;
      uint8_t dxl_id = request->id;
      uint8_t operating_mode = request->operating_mode;
      double goal_current = request->goal_current;
      uint32_t goal_position = request->goal_position;

      RCLCPP_INFO(this->get_logger(), "[Operating Mode: %d] [Goal Current: %f] [Goal Position: %d]", operating_mode, goal_current, goal_position);

      // Disable Torque of DYNAMIXEL
      dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error
      );

      // Write Operating Mode (length : 1 byte)
      dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_OPERATING_MODE,
        operating_mode,
        &dxl_error
      );

      // Enable Torque of DYNAMIXEL
      dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error
      );

      // Current mode
      if (operating_mode == 0) {
        // Set goal current of DYNAMIXEL
        dxl_comm_result = packetHandler->write2ByteTxRx(
          portHandler,
          dxl_id,
          ADDR_GOAL_CURRENT,
          goal_current,
          &dxl_error
        );
      }

      // Position mode
      if (operating_mode == 3) {
        // Write Goal Position (length : 4 bytes)
        // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
        dxl_comm_result = packetHandler->write4ByteTxRx(
          portHandler,
          dxl_id,
          ADDR_GOAL_POSITION,
          goal_position,
          &dxl_error
        );
      }

      // Current based position mode
      if (operating_mode == 5) {
        // Set goal current of DYNAMIXEL
        dxl_comm_result = packetHandler->write2ByteTxRx(
          portHandler,
          dxl_id,
          ADDR_GOAL_CURRENT,
          goal_current,
          &dxl_error
        );

        // Write Goal Position (length : 4 bytes)
        // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
        dxl_comm_result = packetHandler->write4ByteTxRx(
          portHandler,
          dxl_id,
          ADDR_GOAL_POSITION,
          goal_position,
          &dxl_error
        );
      }

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
        response->success = false;
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
        response->success = false;
      } else {
        if (operating_mode == 0) {
          RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Current: %f]", dxl_id, goal_current);
        }
        if (operating_mode == 3) {
          RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", dxl_id, goal_position);
        }
        if (operating_mode == 5) {
          RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Current: %f]", dxl_id, goal_current);
          RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", dxl_id, goal_position);
        }

        response->success = true;
      }
    };

  set_operating_mode_service_ = create_service<SetOperatingMode>("set_operating_mode", set_operating_mode_callback);
}

ReadWriteNode::~ReadWriteNode()
{
}

void ReadWriteNode::publishPresentPosition()
{
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  uint8_t dxl_error = 0;
  uint8_t dxl_id = BROADCAST_ID;
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_PRESENT_POSITION,
    reinterpret_cast<uint32_t *>(&present_position),
    &dxl_error
  );

  if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
    std_msgs::msg::Int64 present_position_msg;
    present_position_msg.data = present_position;
    present_position_publisher_->publish(present_position_msg);
    RCLCPP_INFO(
      this->get_logger(),
      "Published [ID: %d] [Present Position: %d]",
      dxl_id,
      present_position
    );
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to read present position for ID: %d. Error: %s",
      dxl_id,
      packetHandler->getTxRxResult(dxl_comm_result)
    );
  }
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
