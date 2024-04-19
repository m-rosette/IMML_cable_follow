#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
import time
import threading
import numpy as np
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64
from trial_control_msgs.action import RecordData
from trial_control_msgs.srv import LinearActuator, CableState, TactileSlip, TactileGlobal
from dynamixel_control_msgs.srv import SetOperatingMode
from sensor_interfaces.srv import BiasRequest, StartSlipDetection, StopSlipDetection
from sensor_interfaces.msg import SensorState


# Dynamixel current (torque) variables
CURRENT_MODE = 0
UPPER_CURRENT_BOUND = 10.0
LOWER_CURRENT_BOUND = 1.0
OPERATING_CURRENT = 25.0

# Dynamixel position variables
POSITION_MODE = 3
UPPER_POS_BOUND = 3300
LOWER_POS_BOUND = 1290
FULLY_OPEN_POS = 3300
FULLY_CLOSED_POS = 1290

# Dynamixel current based position variables
CURRENT_BASED_POSITION_MODE = 5

# # Gripper V1
# UPPER_POS_BOUND_CB = 103390
# LOWER_POS_BOUND_CB = 101290
# FULLY_OPEN_POS_CB = 101290
# FULLY_CLOSED_POS_CB = 103390

# # # Gripper V2
# UPPER_POS_BOUND_CB = 4725
# LOWER_POS_BOUND_CB = 3250
# FULLY_OPEN_POS_CB = 3250
# FULLY_CLOSED_POS_CB = 4725
UPPER_POS_BOUND_CB = 725
LOWER_POS_BOUND_CB = -800
FULLY_OPEN_POS_CB = -800
FULLY_CLOSED_POS_CB = 725

class GripperControl(Node):
    def __init__(self):
        # Initialize the superclass
        super().__init__('gripper_control')

        self.get_logger().info("Gripper Control node started")

        # Initialize gripper variables
        self.grip_current = 18.0
        self.grip_force_increment = 0.5
        self.cable_state = 1
        self.tactile_0_slipstate = []
        self.tactile_1_slipstate = []   
        self.tactile_0_global_xyz = []
        self.tactile_1_global_xyz = []
        self.declare_parameter("global_x_max", 4.0)   
        self.declare_parameter("global_y_max", 5.0)   
        self.declare_parameter("global_z_max", 7.0)  
        self.global_z_exceeded = False
        self.global_y_exceeded = False    

        # Create record action client and wait for it to start
        self.record_client = ActionClient(self, RecordData, 'record_server')
        self.record_client.wait_for_server()

        # Create publisher for motor current values in a separate thread
        self.motor_current_pub = self.create_publisher(Float64, 'motor_current', 1)
        self.motor_current_thread = threading.Thread(target=self.publish_motor_current)
        self.motor_current_thread.daemon = True
        self.motor_current_thread.start()

        # Create cable status service client
        self.slip_status_client = self.create_client(TactileSlip, 'tactile_slip')
        while not self.slip_status_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for tactile_slip service to start')

        # Create cable status service client
        self.tactile_global_client = self.create_client(TactileGlobal, 'tactile_global')
        while not self.tactile_global_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for tactile_global service to start')

        # Create cable status service client
        self.cable_status_client = self.create_client(CableState, 'cable_pull_jig_state')
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
        self.bias_tactile_client = self.create_client(BiasRequest, '/hub_0/send_bias_request')
        while not self.bias_tactile_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for tactile service to start')

        # Create tactile sensor service client for starting and stoping slip detection
        self.tactile_start_slip_client = self.create_client(StartSlipDetection, '/hub_0/start_slip_detection')
        while not self.tactile_start_slip_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for tactile slip start service to start')

        self.tactile_stop_slip_client = self.create_client(StopSlipDetection, '/hub_0/stop_slip_detection')
        while not self.tactile_stop_slip_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for tactile slip stop service to start')
            
        self.get_logger().info('Tactile slip services started')

    def save_gripper_data(self, filename):
        # Log the status of the user input
        self.get_logger().info(f"Saving data to filename: {filename}")

        # Get instance of action goal
        goal = RecordData.Goal()
        goal.filename = filename

        # Send goal
        self.record_client.send_goal_async(goal)

    def bias_tactile(self):
        self.get_logger().info('Biasing tactile sensors')

        request = BiasRequest.Request()
        self.bias_tactile_client.call_async(request)
    
    def start_slip_detection(self):
        self.get_logger().info("Requesting to start slip detection")

        request = StartSlipDetection.Request()

        self.start_slip_detection_response = self.tactile_start_slip_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.start_slip_detection_response)
        return self.start_slip_detection_response.result()
    
    def stop_slip_detection(self):
        self.get_logger().info("Requesting to stop slip detection")

        request = StopSlipDetection.Request()

        self.stop_slip_detection_response = self.tactile_stop_slip_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.stop_slip_detection_response)
        return self.stop_slip_detection_response.result()
    
    def get_slip_status(self):
        self.get_logger().info("Requesting tactile slip status")

        request = TactileSlip.Request()

        self.slip_status_future = self.slip_status_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.slip_status_future)
        return self.slip_status_future.result()
    
    def get_global_tactile_threshold_status(self):
        self.get_logger().info("Requesting tactile global theshold status")

        request = TactileGlobal.Request()

        self.tactile_global_status_future = self.tactile_global_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.tactile_global_status_future)
        return self.tactile_global_status_future.result()

    def publish_motor_current(self):
        # Function to publish motor current at regular intervals
        while rclpy.ok():
            msg = Float64()
            msg.data = self.grip_current
            self.motor_current_pub.publish(msg)
            time.sleep(0.1) # essentially the publishing rate

    def get_cable_status(self):
        self.get_logger().info("Requesting cable seatment status")

        request = CableState.Request()

        self.cable_state_future = self.cable_status_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.cable_state_future)
        return self.cable_state_future.result()

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

    def run(self, filename):
        global_y_threshold = self.get_parameter('global_y_max').get_parameter_value().double_value
        global_z_threshold = self.get_parameter('global_z_max').get_parameter_value().double_value
        # self.get_logger().info(f"Global y: {global_y_threshold}, Global z {global_z_threshold}")

        # Make sure gripper is open
        # self.grip_current = 25.0
        self.get_logger().info("Opening gripper")
        self.send_motor_request(CURRENT_BASED_POSITION_MODE, 5.0, FULLY_OPEN_POS_CB)
        time.sleep(0.25)

        # Bias the tactile data
        self.bias_tactile()
        time.sleep(0.25)

        # Send the filename to the record action (starts recording)
        self.save_gripper_data(filename)
        time.sleep(2.5)

        # Close the gripper
        self.get_logger().info("Closing gripper")
        self.send_motor_request(CURRENT_BASED_POSITION_MODE, self.grip_current, FULLY_CLOSED_POS_CB)  
        time.sleep(2)

        # Start slip detection controller - Only once tactile sensors have initial contact
        self.get_logger().info("Starting slip detection")
        self.start_detection = self.start_slip_detection()

        # Start cable pull 
        self.get_logger().info("Beginning cable pull")
        self.send_linear_actuator_request(2)

        max_grip = 40

        # Get the current cable state
        self.cable_state = self.get_cable_status()

        while self.cable_state.cable_state == 1:
            self.global_force = self.get_global_tactile_threshold_status()
            self.tactile_0_global_xyz = self.global_force.tactile0_global
            self.tactile_1_global_xyz = self.global_force.tactile1_global
            time.sleep(0.1)

            # self.get_logger().info(f'Sensed global_0_z: {self.tactile_0_global_xyz[2]}, Sensed global_1_z: {self.tactile_1_global_xyz[2]}')
            if self.tactile_0_global_xyz[2] > global_z_threshold or self.tactile_1_global_xyz[2] > global_z_threshold:
                self.global_z_exceeded = True
                self.get_logger().warn("Global tactile Z force threshold exceeded. Terminating cable pull")
                break
            
            if np.abs(self.tactile_0_global_xyz[1]) > global_y_threshold or np.abs(self.tactile_1_global_xyz[1]) > global_y_threshold:
                self.global_y_exceeded = True
                self.get_logger().warn("Global tactile Y force threshold exceeded. Terminating cable pull")
                break

            self.slip_state = self.get_slip_status()
            self.tactile_0_slipstate = self.slip_state.tactile0_slip
            self.tactile_1_slipstate = self.slip_state.tactile1_slip

            # # If slip is detected, increase grip current
            # if self.grip_current > max_grip:
            #     self.get_logger().warn('max grip reached')
            #     break

            if any(slip_num == 3 for slip_num in self.tactile_0_slipstate) or any(slip_num == 3 for slip_num in self.tactile_1_slipstate):
                self.grip_current += self.grip_force_increment
                self.get_logger().info(f"Slip detected [New Current Goal: {self.grip_current}]")
                self.send_motor_request(CURRENT_BASED_POSITION_MODE, self.grip_current, FULLY_CLOSED_POS_CB) 
            
            self.cable_state = self.get_cable_status()

        if self.global_y_exceeded or self.global_z_exceeded:
            # Stop slip detection controller
            self.get_logger().info("Stoping slip detection")
            self.stop_detection = self.stop_slip_detection()

            # Stop the linear actuator
            self.send_linear_actuator_request(4)

            # Open the gripper once home
            self.get_logger().info("Opening gripper")
            self.send_motor_request(CURRENT_BASED_POSITION_MODE, 5.0, FULLY_OPEN_POS_CB)
        
        if self.global_y_exceeded == False and self.global_z_exceeded == False:
            self.get_logger().info("Cable unseated")
            time.sleep(1)

            # Stop slip detection controller
            self.get_logger().info("Stoping slip detection")
            self.stop_detection = self.stop_slip_detection()

            self.get_logger().info("Returning home")
            self.send_linear_actuator_request(3)

            while self.cable_state.cable_state == 0:
                self.cable_state = self.get_cable_status()

            self.get_logger().info("Cable reseated")
            
            # Open the gripper once home
            self.get_logger().info("Opening gripper")
            self.send_motor_request(CURRENT_BASED_POSITION_MODE, 5.0, FULLY_OPEN_POS_CB)


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Make a node class
    gripper_control = GripperControl()

    # Get user input filename
    try:
        filename = str(sys.argv[1])
    except:
        # If no filename is given
        if len(sys.argv) < 2:
            gripper_control.get_logger().warn('No filename given. Proceeding without data collection')
            filename = ''
        # If too many command line inputs are given
        if len(sys.argv) > 2:
            gripper_control.get_logger().error("Usage: ros2 run data_collection <filename>")
            rclpy.shutdown()
            return 

    gripper_control.run(filename)    

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(gripper_control, executor=executor)
    # rclpy.spin(gripper_control)

    # Shutdown everything cleanly
    rclpy.shutdown()


    if __name__ == '__main__':
        main()