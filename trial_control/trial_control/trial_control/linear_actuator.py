#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int64
from trial_control_msgs.srv import LinearActuator, CablePullJigState
import serial
from rclpy.executors import MultiThreadedExecutor


class LinearActuatorControl(Node):
    def __init__(self):
        super().__init__('linear_actuator')
        self.get_logger().info("Starting linear actuator control")

        # Assuming the cable starts seated (1)
        self.cable_state = 1
        self.current_stepper_pos = 0
        self.jig_status_list = []

        # Create publisher for cable jig status
        self.cable_status_pub = self.create_publisher(Int16, 'cable_status', 10)
        self.stepper_pos_pub = self.create_publisher(Int64, 'stepper_pos', 10)

        self.cable_pull_jig_timer = self.create_timer(0.005, self.cable_pull_jig_timer_callback)

        # Create services
        self.linear_actuator_srv = self.create_service(LinearActuator, 'linear_actuator_control', self.linear_actuator_callback)
        self.cable_pull_jig_state = self.create_service(CablePullJigState, 'cable_pull_jig_state', self.cable_pull_jig_state_callback)

        # Define Arduino serial port and baud rate
        self.arduino_port = '/dev/ttyACM1'
        self.baudrate = 115200

        # Open the serial connection to Arduino
        self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)

    def linear_actuator_callback(self, request, response):
        self.get_logger().info('Received movement request')
        try:
            # Update response
            response.success = True
            move_dir = request.move_direction
            self.get_logger().info(f"Requesting movement to {request.move_direction}")

            # Send command to arduino
            self.arduino.write(move_dir.to_bytes(2, byteorder='little'))

        except ValueError:
            response.success = False

        return response
    
    def cable_pull_jig_timer_callback(self):
        if self.arduino.in_waiting > 0:
            cable_pull_jig_status = self.arduino.read(size=self.arduino.in_waiting).strip().split(b'\r\n')

            # Append the latest two readings and pop the rest - helps catch and missreadings
            self.jig_status_list.append(cable_pull_jig_status)
            if len(self.jig_status_list) >= 3:
                self.jig_status_list = self.jig_status_list[-2:]

            try:
                jig_status = self.jig_status_list[0]
                cable_status_coded, current_stepper_pos_coded = jig_status[-2].decode().split(',')
                self.cable_state = int(cable_status_coded)
                self.current_stepper_pos = int(current_stepper_pos_coded)
                print(self.cable_state)
            except IndexError:
                jig_status = self.jig_status_list[1]
                cable_status_coded, current_stepper_pos_coded = jig_status[-2].decode().split(',')
                self.cable_state = int(cable_status_coded)
                self.current_stepper_pos = int(current_stepper_pos_coded)
                print(self.cable_state)

            self.cable_status_pub_callback()
            self.stepper_pos_pub_callback()

    def cable_pull_jig_state_callback(self, request, response):
        self.get_logger().info("Cable pull jig status requested")
        response.cable_state = self.cable_state
        response.stepper_position = self.current_stepper_pos
        return response
    
    def cable_status_pub_callback(self):
        msg = Int16()
        msg.data = self.cable_state
        self.cable_status_pub.publish(msg)

    def stepper_pos_pub_callback(self):
        msg = Int64()
        msg.data = self.current_stepper_pos
        self.stepper_pos_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    linear_actuator = LinearActuatorControl()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(linear_actuator, executor=executor)
    # rclpy.spin(linear_actuator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
