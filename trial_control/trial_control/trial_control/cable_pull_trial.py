#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import sys
import os
import time

from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from trial_control_msgs.action import RecordData
from trial_control_msgs.srv import LinearActuator, CableState
from dynamixel_control_msgs.srv import SetOperatingMode
from sensor_interfaces.srv import BiasRequest, StartSlipDetection, StopSlipDetection
from sensor_interfaces.msg import SensorState
# from trial_control_msgs.msg import CableState


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
LOWER_POS_BOUND_CB = 101290
FULLY_OPEN_POS_CB = 101290
FULLY_CLOSED_POS_CB = 103390

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
        self.motor_current_pub = self.create_publisher(Float64, 'motor_current', 1)
        self.motor_current_timer = self.create_timer(0.1, self.motor_current_callback)

        # # Create cable state subscription
        # self.cable_state_sub = self.create_subscription(CableState, 'cable_state', self.cable_state_callback, 10)

        # Create cable status service client
        self.cable_status_client = self.create_client(CableState, 'cable_state')
        while not self.cable_status_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for cable_status service to start')

        # Create dynamixel service client
        self.dynamixel_client = self.create_client(SetOperatingMode, 'set_operating_mode')
        while not self.dynamixel_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for dynamixel service to start')

        # Create linear actuator service client
        self.linear_actuator_client = self.create_client(LinearActuator, 'linear_actuator_control')
        while not self.linear_actuator_client.wait_for_service(timeout_sec=1):
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

    def get_cable_status(self):
        self.get_logger().info("Requesting cable seatment status")

        request = CableState.Request()

        self.cable_state = self.cable_status_client.call_async(request)

    def send_linear_actuator_request(self, command):
        self.get_logger().info(f"Requested linear actuator position: {command}")

        # Get instance of srv
        request = LinearActuator.Request()
        request.move_direction = command

        # Send request
        self.response = self.linear_actuator_client.call_async(request)

    def send_motor_request(self, operating_mode, goal_current, goal_position):
        # Get instance of srv
        request = SetOperatingMode.Request()
        request.id = 1
        request.operating_mode = operating_mode

        bounded_goal_current, bounded_goal_position = self.bound_motor_input(operating_mode, goal_current, goal_position)

        self.get_logger().info(f"Dynamixel Request: [Operating mode: {operating_mode}] [Goal Current: {bounded_goal_current}] [Goal Position: {bounded_goal_position}]")

        request.goal_current = bounded_goal_current
        request.goal_position = bounded_goal_position

        # Send request
        self.response = self.dynamixel_client.call_async(request)

    def bound_motor_input(self, operating_mode, goal_current, goal_position):
        # Check the input position bounds
        if operating_mode == POSITION_MODE and goal_position > UPPER_POS_BOUND:
            self.get_logger().warn(F"Position upper bound exceeded - capping value. [Goal Position: {UPPER_POS_BOUND}]")
            goal_position = UPPER_POS_BOUND
        elif operating_mode == POSITION_MODE and goal_position < LOWER_POS_BOUND:
            self.get_logger().warn(F"Position lower bound exceeded - capping value. [Goal Position: {LOWER_POS_BOUND}]")
            goal_position = LOWER_POS_BOUND

        # Check the input current bounds
        if operating_mode == CURRENT_MODE and goal_current > UPPER_CURRENT_BOUND:
            self.get_logger().warn(F"Current upper bound exceeded - capping value. [Goal Current: {UPPER_CURRENT_BOUND}]")
            goal_current = UPPER_CURRENT_BOUND
        elif operating_mode == CURRENT_MODE and goal_current < LOWER_CURRENT_BOUND:
            self.get_logger().warn(F"Current lower bound exceeded - capping value. [Goal Current: {LOWER_CURRENT_BOUND}]")
            goal_current = LOWER_CURRENT_BOUND

        # Check the input position bounds (current based position)
        if operating_mode == CURRENT_BASED_POSITION_MODE and goal_position > UPPER_POS_BOUND_CB:
            self.get_logger().warn(F"Current based position upper bound exceeded - capping value. [Goal Position: {UPPER_POS_BOUND_CB}]")
            goal_position = UPPER_POS_BOUND_CB
        elif operating_mode == CURRENT_BASED_POSITION_MODE and goal_position < LOWER_POS_BOUND_CB:
            self.get_logger().warn(F"Current based position lower bound exceeded - capping value. [Goal Position: {LOWER_POS_BOUND_CB}]")
            goal_position = LOWER_POS_BOUND_CB

        return goal_current, goal_position
    
    def cable_state_callback(self, msg):
        self.cable_state = msg.cable_state
        self.get_logger().info(f'Cable callback with state: {self.cable_state}')

    def run(self, filename):
        # Send the filename to the record action (starts recording)
        self.save_tactile_data(filename)

        # Make sure gripper is open
        self.grip_current = OPERATING_CURRENT
        self.get_logger().info("Opening gripper")
        self.send_motor_request(CURRENT_BASED_POSITION_MODE, self.grip_current, FULLY_OPEN_POS_CB)

        time.sleep(2.5)

        # Close the gripper
        self.get_logger().info("Closing gripper")
        self.send_motor_request(CURRENT_BASED_POSITION_MODE, self.grip_current, FULLY_CLOSED_POS_CB)  

        # Start slip detection controller - Only once tactile sensors have initial contact
        self.get_logger().info("Starting slip detection")
        # request = Empty.Request()
        request = StartSlipDetection.Request()
        self.tactile_start_slip_client.call_async(request)

        # Start cable pull 
        self.get_logger().info("Beginning cable pull")
        
        # Move linear actuator forward
        self.send_linear_actuator_request('PULL')
        # self.get_logger().info(f'Cable state: {self.cable_state}')
        time.sleep(1.5)

        while self.cable_state == 'SEATED':
            self.get_cable_status()
            
            self.get_logger().info(f'Cable state: {self.cable_state}')
            # If slip is detected, increase grip current
            if any(slip_num == 3 for slip_num in self.tactile_0_slipstate) or any(slip_num == 3 for slip_num in self.tactile_1_slipstate):
                grip_current += 0.1
                self.get_logger().info(f"Slip detected [New Current Goal: {grip_current}]")
                self.send_motor_request(CURRENT_BASED_POSITION_MODE, grip_current, FULLY_CLOSED_POS_CB) 
                # time.sleep(0.5)
        
        self.get_logger().info("Cable unseated")

        # Stop slip detection controller
        self.get_logger().info("Stoping slip detection")
        # request = Empty.Request()
        request = StopSlipDetection.Request()
        self.tactile_stop_slip_client.call_async(request)

        self.get_logger().info("Returning home")
        # while self.cable_state == 'UNSEATED':
        #     # Move linear actuator home
        self.send_linear_actuator_request('HOME')
        
        # Open the gripper once home
        self.get_logger().info("Opening gripper")
        self.send_motor_request(CURRENT_BASED_POSITION_MODE, OPERATING_CURRENT, FULLY_OPEN_POS_CB)


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Make a node class
    gripper_control = GripperControl()

    # # Use a MultiThreadedExecutor to enable processing goals concurrently
    # executor = MultiThreadedExecutor()

    # get user input filename
    try:
        filename = str(sys.argv[1])
    except:
        # If no filename is given or too many command line inputs 
        if len(sys.argv) != 2:
            gripper_control.get_logger().error("Usage: ros2 run data_collection <filename>")
            rclpy.shutdown()
            return 

    gripper_control.run(filename)    

    rclpy.spin(gripper_control)
    # rclpy.spin(gripper_control, executor=executor)

    # Shutdown everything cleanly
    rclpy.shutdown()


    if __name__ == '__main__':
        main()