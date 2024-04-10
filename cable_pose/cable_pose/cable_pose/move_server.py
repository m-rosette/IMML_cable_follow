#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
import cv2
from cv_bridge import CvBridge, CvBridgeError
from process_results import ResultsProcessor

from sensor_msgs.msg import Image
from cable_pose_msgs.srv import MoveData

from dataset_info import get_dataset_info


class MoveServer(Node):
    def __init__(self):
        super().__init__('move_server')

        self.get_logger().info("Starting move server")

        self.bridge = CvBridge()

        self.dataset_info = get_dataset_info()

        self.num_features = self.dataset_info["num_features"]
        self.num_time_steps = self.dataset_info["time_steps_to_use"]
        self.operation_data_path = self.dataset_info["operation_data_path"]
        self.operation_image_path = self.dataset_info["operation_image_path"]

        self.results_processor = ResultsProcessor()

        self.image_pub = self.create_publisher(Image, 'annotated_image', 10)

        self.move_data_service = self.create_service(MoveData, 'gripper_move_data', self.load_data_callback)

    def receive_data_service(self, request, response):

        x_data = request.gripper_data
        self.move_right = request.move_right
        x_data = np.array(x_data).reshape(1, self.num_time_steps, self.num_features)[0]
        image = request.image

        return self.results_processor.process_result(x_data, image)

    def load_data_callback(self, request, response):

        trial_num = str(int(request.gripper_data[0]))

        path_to_data = "/media/jostan/portabits/kyle/swapped/" + trial_num + "_swapped.csv"

        path_to_image = "/media/jostan/portabits/kyle/" + trial_num + ".jpg"

        move_right = request.move_right

        gripper_data = pd.read_csv(path_to_data)

        # Extract the last 30 rows and drop non-feature columns if necessary
        x_data = gripper_data.iloc[-self.dataset_info['time_steps_to_use']:].drop(
            columns=self.dataset_info['non_feature_columns']).values

        # Reshape the data to be 3D
        x_data = x_data.reshape(1, self.num_time_steps, self.num_features)

        image = cv2.imread(path_to_image)

        prediction = self.results_processor.get_result(x_data)[0]

        move_destination, move_angle, image = self.results_processor.process_result(prediction, image, move_right)

        response = MoveData.Response()
        response.x = move_destination[0]
        response.y = move_destination[1]
        response.angle = move_angle

        # publish the image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        return response
    

def main(args=None):
	rclpy.init(args=args)
	move_service = MoveServer()
	rclpy.spin(move_service)
	rclpy.shutdown()
    

if __name__ == "__main__":
    main()

