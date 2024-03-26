#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
import os
import time

from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from trial_control_msgs.action import RecordData
from trial_control_msgs.srv import LinearActuator
from dynamixel_control_msgs.srv import SetOperatingMode
from sensor_interfaces.srv import BiasRequest, StartSlipDetection, StopSlipDetection
from sensor_interfaces.msg import SensorState


# Dynamixel current (torque) variables
CURRENT_MODE = 0
UPPER_CURRENT_BOUND = 10.0
LOWER_CURRENT_BOUND = 1.0
OPERATING_CURRENT = 4.0

# Dynamixel position variables
POSITION_MODE = 3
UPPER_POS_BOUND = 3300
LOWER_POS_BOUND = 1290
FULLY_OPEN_POS = 3300
FULLY_CLOSED_POS = 1290

# Dynamixel current based position variables
CURRENT_BASED_POSITION_MODE = 5
UPPER_POS_BOUND_CB = 103390
LOWER_POS_BOUND_CB = 101280


class GripperControl(Node):
    def __init__(self):
        # Initialize the superclass
        super().__init__('gripper_control')

        self.get_logger().info("Gripper Control node started")

        # Create record action client
        self.record_client = ActionClient(self, RecordData, 'record_server')

        # Wait until the server is ready to accept an action request
        self.record_client.wait_for_server()

        # Create publisher for operating current values
        self.motor_current_pub = self.create_publisher(Float64, 'motor_current', 10)
        self.motor_current_timer = self.create_timer(0.1, self.motor_current_callback)

        # Create cable state subscription
        self.cable_state_sub = self.create_subscription(String, 'cable_state', self.cable_state_callback, 10)

        # Create dynamixel service client
        self.dynamixel_client = self.create_client(SetOperatingMode, 'set_operating_mode')
        while not self.dynamixel_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for dynamixel service to start')

        # Create linear actuator service client
        self.linear_actuator_client = self.create_client(LinearActuator, 'linear_actuator_control')
        while not self.dynamixel_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for linear_actuator service to start')

        # Create tactile sensor service client to send bias request (zero initial readings)
        self.tactile_client = self.create_client(BiasRequest, '/hub_0/send_bias_request')
        while not self.tactile_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for tactile service to start')
        self.get_logger().info('Biasing tactile sensors')

        # Create tactile sensor service client for starting and stoping slip detection
        self.tactile_start_slip_client = self.create_client(StartSlipDetection, '/hub_0/start_slip_detection')
        while not self.tactile_start_slip_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for tactile slip start service to start')

        self.tactile_stop_slip_client = self.create_client(StopSlipDetection, '/hub_0/stop_slip_detection')
        while not self.tactile_stop_slip_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for tactile slip stop service to start')
            
        self.get_logger().info('Tactile slip services started')

        # Subscribe to tactile sensor feedback
        self.tactile_0_sub = self.create_subscription(SensorState, 'hub_0/sensor_0', self.tactile_0_callback, 10)
        self.tactile_1_sub = self.create_subscription(SensorState, 'hub_0/sensor_1', self.tactile_1_callback, 10)
        self.tactile_0_slipstate = {}
        self.tactile_1_slipstate = {}

        self.grip_current = 0
    
        # Initialize cable state
        self.cable_state = 'SEATED'

    def save_tactile_data(self, filename):
        # Log the status of the user input
        self.get_logger().info(f"Saving data to filename: {filename}")

        # Get instance of action goal
        goal = RecordData.Goal()
        goal.filename = filename

        # Send goal
        self.record_client.send_goal_async(goal)
    
    def tactile_0_callback(self, tac_msg):
        for i in range(9):
            self.tactile_0_slipstate[f'0_slipstate_{i}'] = tac_msg.pillars[i].slip_state

    def tactile_1_callback(self, tac_msg):
        for i in range(9):
            self.tactile_1_slipstate[f'1_slipstate_{i}'] = tac_msg.pillars[i].slip_state

    def motor_current_callback(self):
        msg = Float64()
        msg.data = self.grip_current
        self.motor_current_pub.publish(msg)

    def send_linear_actuator_request(self, target_position):
        self.get_logger().info(f"Requested linear actuator position: {target_position}")

        # Get instance of srv
        request = LinearActuator.Request()
        request.location_goal = target_position

        # Send request
        self.response = self.linear_actuator_client.call_async(request)

    def send_motor_request(self, operating_mode, operation_target):
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

        # Check the input position bounds (current based position)
        if operating_mode == CURRENT_BASED_POSITION_MODE and operation_target > UPPER_POS_BOUND_CB:
            self.get_logger().warn(F"Current based position upper bound exceeded - capping value. [Goal Position: {UPPER_POS_BOUND}]")
            operation_target = UPPER_POS_BOUND_CB
        elif operating_mode == CURRENT_BASED_POSITION_MODE and operation_target < LOWER_POS_BOUND_CB:
            self.get_logger().warn(F"Current based position lower bound exceeded - capping value. [Goal Position: {LOWER_POS_BOUND}]")
            operation_target = LOWER_POS_BOUND_CB

        return operation_target
    
    def cable_state_callback(self, msg):
        self.cable_state = msg.cable_state


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

    # Make sure gripper is open
    gripper_control.grip_current = OPERATING_CURRENT
    gripper_control.get_logger().info("Opening gripper")
    gripper_control.send_motor_request(CURRENT_BASED_POSITION_MODE, gripper_control.grip_current, FULLY_OPEN_POS)

    # Close the gripper
    gripper_control.get_logger().info("Closing gripper")
    gripper_control.send_motor_request(CURRENT_BASED_POSITION_MODE, gripper_control.grip_current, FULLY_CLOSED_POS)  

    # Start slip detection controller - Only once tactile sensors have initial contact
    gripper_control.get_logger().info("Starting slip detection")
    request = Empty.Request()
    gripper_control.tactile_start_slip_client.call_async(request)

    # Start cable pull 
    gripper_control.get_logger().info("Beginning cable pull")
    while gripper_control.cable_state == 'SEATED':
        # Move linear actuator forward
        gripper_control.send_linear_actuator_request('PULL')

        # If slip is detected, increase grip current
        if any(slip_num == 3 for slip_num in gripper_control.tactile_0_slipstate) or any(slip_num == 3 for slip_num in gripper_control.tactile_1_slipstate):
            grip_current += 0.1
            gripper_control.get_logger().info(f"Slip detected [New Current Goal: {grip_current}]")
            gripper_control.send_motor_request(CURRENT_BASED_POSITION_MODE, grip_current, FULLY_CLOSED_POS) 
    
    gripper_control.get_logger().info("Cable unseated")

    # Stop slip detection controller
    gripper_control.get_logger().info("Stoping slip detection")
    request = Empty.Request()
    gripper_control.tactile_stop_slip_client.call_async(request)

    gripper_control.get_logger().info("Returning home")
    while gripper_control.cable_state == 'UNSEATED':
        # Move linear actuator home
        gripper_control.send_linear_actuator_request('HOME')
    
    # Open the gripper once home
    gripper_control.get_logger().info("Opening gripper")
    gripper_control.send_motor_request(CURRENT_BASED_POSITION_MODE, OPERATING_CURRENT, FULLY_OPEN_POS)

    # Shutdown everything cleanly
    rclpy.shutdown()


    if __name__ == '__main__':
        main()