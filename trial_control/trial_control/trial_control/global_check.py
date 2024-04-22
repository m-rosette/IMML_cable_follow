#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from trial_control_msgs.srv import TactileGlobal
from sensor_interfaces.msg import SensorState


class TactileGlobalCheck(Node):
    def __init__(self):
        super().__init__('tactile_global_check')
        self.get_logger().info("Starting tactile global check")

        self.tactile_0_global = []
        self.tactile_1_global = []

        # Subscribe to tactile sensor feedback
        self.tactile_0_sub = self.create_subscription(SensorState, 'hub_0/sensor_0', self.tactile_0_callback, 10)
        self.tactile_1_sub = self.create_subscription(SensorState, 'hub_0/sensor_1', self.tactile_1_callback, 10)

        # Create service to get the tactile global forces/torques
        self.tactile_global_srv = self.create_service(TactileGlobal, 'tactile_global', self.tactile_global_callback)

    def tactile_0_callback(self, tac_msg):
        self.tactile_0_global = [tac_msg.gfx, tac_msg.gfy, tac_msg.gfz]

    def tactile_1_callback(self, tac_msg):
        self.tactile_1_global = [tac_msg.gfx, tac_msg.gfy, tac_msg.gfz]
    
    def tactile_global_callback(self, request, response):
        self.get_logger().info('Recieved global threshold request')
        try:
            # Update response
            response.tactile0_global = self.tactile_0_global
            response.tactile1_global = self.tactile_1_global

        except ValueError:
            pass

        return response
        

def main(args=None):
    rclpy.init(args=args)
    tactile_global = TactileGlobalCheck()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(tactile_global, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()