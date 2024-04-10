#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
import os

from cable_pose_msgs.srv import MoveData

from dataset_info import get_dataset_info


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')

        self.server_connection = self.create_client(MoveData, 'gripper_move_data')
        
        # Wait until we have a connection to the server.
        while not self.server_connection.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for service to start')
                  
    def send_request(self, num_files):
        for i in range(num_files):
            # file_path = test_data_file_paths[i]
            # trial_num = float(file_path.split("_")[0])images
            # sample_num = float(file_path.split("_")[1])
            # print("Trial: {}, Sample: {}".format(trial_num, sample_num))

            request = MoveData.Request()

            # request.gripper_data = x_data
            request.gripper_data = [i, 5.9]

            request.move_right = False

            self.response = self.server_connection.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    client = MoveClient()
     
    test_data_file_paths = os.listdir("/media/jostan/portabits/kyle")
    num_files = int(len(test_data_file_paths)/2)

    client.send_request(num_files)

    if client.response.done():
        try:
            # We get the response through result()
            answer = client.response.result()
        except Exception as e:
            # An exception will be thrown if things fail.
            client.get_logger().info('Service call failed: {0}'.format(e))
        else:
            # Then, we're going to log the result.
            client.get_logger().info(f'response: x: {answer.x}, y: {answer.y}, z: {answer.angle}')

	# Shut things down when we're done.
    rclpy.shutdown()


# This is the entry point if we run this node as an executable.
if __name__ == '__main__':
	main()


    