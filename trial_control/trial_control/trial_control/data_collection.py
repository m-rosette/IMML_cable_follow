#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
import os
import time

from trial_control_msgs.action import RecordData
from trial_control_msgs.srv import LinearActuator
from dynamixel_control_msgs.srv import SetOperatingMode
from sensor_interfaces.srv import BiasRequest


# Linear actuator positions (value: negative = forward, positive = backward)
BACKWARD_POS = 0
FORWARD_POS = -30000

# Dynamixel current (torque) variables
CURRENT_MODE = 0
CLOSE_CURRENT = 70
UPPER_CURRENT_BOUND = 100
LOWER_CURRENT_BOUND = 10

# Dynamixel position variables
POSITION_MODE = 3
UPPER_POS_BOUND = 500
LOWER_POS_BOUND = 50
MAX_POS = 4075
FULLY_OPEN_POS = 50


class GripperControl(Node):
    def __init__(self):
        # Initialize the superclass
        super().__init__('gripper_control')

        self.get_logger().info("Gripper Control node started")

        # Create record action client
        self.record_client = ActionClient(self, RecordData, 'record_server')

        # Wait until the server is ready to accept an action request
        self.record_client.wait_for_server()

        # Create dynamixel service client
        self.dynamixel_client = self.create_client(SetOperatingMode, 'set_operating_mode')
        while not self.dynamixel_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for dynamixel service to start')

        # Create linear actuator service client
        self.linear_actuator_client = self.create_client(LinearActuator, 'input_number')
        while not self.dynamixel_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for dynamixel service to start')

        # Create tactile sensor service client to send bias request (zero initial readings)
        self.tactile_client = self.create_client(BiasRequest, '/hub_0/send_bias_request')
        while not self.tactile_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for tactile service to start')
        self.get_logger().info('Biasing tactile sensors')
    
    def save_tactile_data(self, filename):
        # Log the status of the user input
        self.get_logger().info(f"Saving data to filename: {filename}")

        # Get instance of action goal
        goal = RecordData.Goal()
        goal.filename = filename

        # Send goal
        self.record_client.send_goal_async(goal)

    def send_linear_actuator_request(self, target_position):
        self.get_logger().info(f"Requested linear actuator position: {target_position}")

        # Get instance of srv
        request = LinearActuator.Request()
        request.location_goal = target_position

        # Send request
        self.response = self.linear_actuator_client.call_async(request)

    def send_dual_motor_request(self, operating_mode, operation_target):
        # Get instance of srv
        request = SetOperatingMode.Request()
        request.id = 1
        request.operating_mode = operating_mode

        bounded_operation_target = self.bound_motor_input(operating_mode, operation_target)

        self.get_logger().info(f"Dynamixel Request: [Operating mode: {operating_mode}] [Operation target: {bounded_operation_target}]")

        request.operation_target = bounded_operation_target

        # Send request
        self.response = self.dynamixel_client.call_async(request)

    def bound_motor_input(self, operating_mode, operation_target):
        # Check the input position bounds
        if operating_mode == POSITION_MODE and operation_target > UPPER_POS_BOUND:
            self.get_logger().warn(F"Position upper bound exceeded - capping value. [Goal Position: {UPPER_POS_BOUND}]")
            operation_target = UPPER_POS_BOUND
        elif operating_mode == POSITION_MODE and operation_target < LOWER_POS_BOUND:
            self.get_logger().warn(F"Position lower bound exceeded - capping value. [Goal Position: {LOWER_POS_BOUND}]")
            operation_target = LOWER_POS_BOUND

        # Check the input current bounds
        if operating_mode == CURRENT_MODE and operation_target > UPPER_CURRENT_BOUND:
            self.get_logger().warn(F"Current upper bound exceeded - capping value. [Goal Current: {UPPER_CURRENT_BOUND}]")
            operation_target = UPPER_CURRENT_BOUND
        elif operating_mode == CURRENT_MODE and operation_target < LOWER_CURRENT_BOUND:
            self.get_logger().warn(F"Current lower bound exceeded - capping value. [Goal Current: {LOWER_CURRENT_BOUND}]")
            operation_target = LOWER_CURRENT_BOUND

        return operation_target


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Make a node class
    gripper_control = GripperControl()

    # get user input filename
    try:
        filename = str(sys.argv[1])
    except:
        # If no filename is given or too many command line inputs 
        if len(sys.argv) != 2:
            gripper_control.get_logger().error("Usage: ros2 run data_collection <filename>")
            rclpy.shutdown()
            return 

    # Send the filename to the record action (starts recording)
    gripper_control.save_tactile_data(filename)

    # Make sure gripper is open (position mode)
    gripper_control.get_logger().info("Opening gripper")
    gripper_control.send_dual_motor_request(POSITION_MODE, FULLY_OPEN_POS)

    # Move linear actuator forward
    gripper_control.get_logger().info("Moving linear actuator forward")
    gripper_control.send_linear_actuator_request(-20000)

    # Allow time for the linear actuator to move
    time.sleep(15)

    # Close gripper (current mode)
    gripper_control.get_logger().info("Closing gripper")
    gripper_control.send_dual_motor_request(CURRENT_MODE, CLOSE_CURRENT)

    # Move linear actuator backward
    gripper_control.get_logger().info("Moving linear actuator backward")
    gripper_control.send_linear_actuator_request(0)

    # Allow time for the linear actuator to move
    time.sleep(15)

    # Open gripper (position mode)
    gripper_control.get_logger().info("Opening gripper")
    gripper_control.send_dual_motor_request(POSITION_MODE, FULLY_OPEN_POS)

    # Shutdown everything cleanly
    rclpy.shutdown()


    if __name__ == '__main__':
        main()