#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys

from gripper_msgs.action import RecordData


class RecordClient(Node):
	def __init__(self):
		# Initialize the superclass
		super().__init__('record_client')

		# Creat action client
		self.client = ActionClient(self, RecordData, 'record_server')

	def send_goal(self, goal_filename):
		goal = RecordData.Goal()
		goal.filename = goal_filename

		# Wait until the server is ready to accept an action request
		self.client.wait_for_server()

		# Make the action request
		self.result = self.client.send_goal_async(goal, feedback_callback=self.feedback)

		# Attach a callback to the call, so that we can react when the action is accepted or
		# rejected.
		self.result.add_done_callback(self.done)

	# Process feedback as it comes in.
	def feedback(self, feedback):
		# DO SOMETHING WITH THE FEEDBACK HERE
		pass

	# This callback fires when the action is accepted or rejected
	def done(self, future):
		# Get the result of requesting the action
		goal_handle = future.result()

		# Handle action rejection
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected')
			return

		# Handle action acceptance
		self.get_logger().info('Goal accepted')
		self.result_handle = goal_handle.get_result_async()
		self.result_handle.add_done_callback(self.process_result)


	# This callback fires when there are results to be had
	def process_result(self, future):
		# Get the result
		result = future.result().result
		# filename = future.goal().filename

		# Log the result to the info channel.
		self.get_logger().info(f'Saved file: {result.result}')
		
		rclpy.shutdown()
		

def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)
	
	# get goal filename, if available
	try:
		filename = str(sys.argv[1])
	except:
		filename = "None"

	client = RecordClient()

	client.send_goal(filename)

	rclpy.spin(client)


if __name__ == '__main__':
	main()
