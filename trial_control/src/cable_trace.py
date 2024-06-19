#!/usr/bin/env python3

import rospy
import threading
import time
from trial_control.msg import MoveAction, MoveGoal, Movement, RecordGoal, RecordAction, PullGoal, PullAction
from actionlib import SimpleActionClient
from std_msgs.msg import String, Int16
from lbc_project.srv import MoveData, MoveDataResponse
from trial_control.srv import PictureTrigger, PictureTriggerResponse, CablePullTrigger, CablePullTriggerResponse
from papillarray_ros_v2.srv import *
import os
"""
1) Freedrive the robot to the the starting location on the cable

Repeat:
A) Close the gripper, recording data
B) Open the gripper
C) Pass the data to the LSTM service
D) Get back the delta x, y, theta 
E) Call the move_arm action server

"""


class CableTrace:
    def __init__(self):
        self.storage_directory = '/data/grip_data/'
        self.file_num = 0
        self.pull_triggered = False
        # Set up the movement action client
        self.move_ac = SimpleActionClient("move_server", MoveAction)
        rospy.loginfo("Waiting for movement server to come up.")
        self.move_ac.wait_for_server()
        rospy.loginfo("Movement server up!")

        # Setup the recording action client
        self.record_ac = SimpleActionClient("record_server", RecordAction)
        rospy.loginfo("Waiting for recording server to come up.")
        self.record_ac.wait_for_server()
        rospy.loginfo("Recording server up!")

        rospy.wait_for_service('picture_trigger')
        rospy.loginfo("Picture service up, ready!")
        self.pic = rospy.ServiceProxy('picture_trigger', PictureTrigger)
        
        # Create the cable pull service server
        self.cable_pull_service = rospy.Service('cable_pull_trigger', CablePullTrigger, self.handle_cable_pull_trigger)

        # Set up the gripper position/current publisher
        self.gripper_pos_pub = rospy.Publisher('Gripper_Cmd', String, queue_size=5)

        # rospy.wait_for_service('gripper_move_data')
        # rospy.loginfo("LSTM service up, ready!")
        # self.lstm = rospy.ServiceProxy('gripper_move_data', MoveData)

        # Create the cable pull action client
        self.pull_action = SimpleActionClient("pull_server", PullAction)
        rospy.loginfo("Waiting for pull action server to come up.")
        self.pull_action.wait_for_server()
        rospy.loginfo("Pull action server up!")

        # Bias tactile sensors
        self.bias_request_srv_client()
        rospy.loginfo("Biasing tactile sensors")

    def bias_request_srv_client(self):
        rospy.wait_for_service('/hub_0/send_bias_request')
        srv = rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)
        success = srv()
        return success

    def send_pull_goal(self):
        pull_goal = PullGoal()  # sending an empty goal
        self.pull_action.send_goal(pull_goal)
        self.pull_action.wait_for_result(timeout=rospy.Duration(10))
        pull_result = self.pull_action.get_result()
        if pull_result:
            rospy.loginfo(f"Pull action completed with result: {pull_result.end_condition}")
        else:
            rospy.logwarn("Pull action did not finish within the timeout.")
            self.pull_action.cancel_all_goals()
            self.cancel_move_goal()

    def send_move_goal(self):
        movement = Movement(dx=-50, dy=0, dtheta=0, vel=0.005)
        goal = MoveGoal(delta_move=movement)
        self.move_ac.send_goal(goal)
        self.move_ac.wait_for_result(timeout=rospy.Duration(10))
        move_result = self.move_ac.get_result()
        if move_result:
            rospy.loginfo(f"Move action completed with result: {move_result}")
        else:
            rospy.logwarn("Move action did not finish within the timeout.")
            self.move_ac.cancel_all_goals()

    def handle_cable_pull_trigger(self, req):
        rospy.loginfo("Cable pull service called")

        self.pull_triggered = True

        # Create threads for pull and move goals
        pull_thread = threading.Thread(target=self.send_pull_goal)
        move_thread = threading.Thread(target=self.send_move_goal)

        # Start both threads
        pull_thread.start()
        move_thread.start()

        # Wait for both threads to finish
        pull_thread.join()
        # time.sleep(0.25)
        move_thread.join()

        # Return a response to the service call
        return CablePullTriggerResponse(success=True)

    def cancel_move_goal(self):
        rospy.loginfo("Canceling move action goal")
        self.move_ac.cancel_all_goals()

    # def handle_cable_pull_trigger(self, req):
    #     rospy.loginfo("Cable pull service called")

    #     self.pull_triggered = True
        
    #     # Send goal to pull action server
    #     pull_goal = PullGoal() # sending an empty goal
    #     self.pull_action.send_goal(pull_goal)

    #     movement = Movement(dx = -20, dy = 0, dtheta = 0, vel=0.01)
    #     # movement = Movement(dx = 0, dy = 0, dtheta = 45)
    #     goal = MoveGoal(delta_move=movement)
    #     self.move_ac.send_goal(goal)

    #     self.pull_action.wait_for_result(timeout=rospy.Duration(10))
    #     # assuming waiting for the pull action handles the waiting for arm movement
        
    #     pull_result = self.pull_action.get_result()
    #     # rospy.loginfo(f"Pull action completed with result: {pull_result.end_condition}")

    #     self.move_ac.cancel_all_goals()

    #     # Return a response to the service call
    #     return CablePullTriggerResponse(success=True)

    def main(self):
        # Open gripper
        self.gripper_pos_pub.publish("position_3300")

        # Drive robot to start
        _ = input("Freedrive robot to start, then start program, then hit enter.")
        self.file_num = self.get_start_file_index()
        # self.file_num = 0

        while not rospy.is_shutdown():
            # Bias the sensors every time the gripper is open
            self.bias_request_srv_client()

            if self.pull_triggered:
                rospy.loginfo(f"Cable pull controller activated")
                break

            print("FILE NUM: ",self.file_num)
            goal = RecordGoal(file_name=str(self.file_num))
            self.record_ac.send_goal(goal)

            # Close gripper
            self.gripper_pos_pub.publish("current_3")
            rospy.sleep(2.5) # Sleep a tiny bit then stop recording
            
            self.record_ac.wait_for_result()
            result = self.record_ac.get_result()

            # Take a picture
            picture_return = self.pic(trial_num = str(self.file_num))

            # Open the gripper
            self.gripper_pos_pub.publish("position_3300")

            # # TODO: Call LSTM service
            # move_return = self.lstm(gripper_data = result.data, move_right=True)

            # print(move_return)

            # Move robot with movement from LSTM
            # movement = Movement(dx = - move_return.x, dy = move_return.y, dtheta = move_return.angle)
            # # movement = Movement(dx = - move_return.y, dy = move_return.x, dtheta = move_return.angle)
            # movement = Movement(dx = 0, dy = 0, dtheta = 20)
            # goal = MoveGoal(delta_move=movement)
            # self.move_ac.send_goal(goal)
            # self.move_ac.wait_for_result()
            input("Enter to continue to next step")

            self.file_num += 1

    def get_start_file_index(self):
        # Returns the starting file number (old number)
        current_files = os.listdir(self.storage_directory)
        try:
            numbers = np.array([i.split('.')[0] for i in current_files], dtype=int)
            print(numbers)
            return np.max(numbers) + 1
        except Exception as e:
            return 0


if __name__ == '__main__':
    rospy.init_node('cable_trace', anonymous=True)
    cable_trace = CableTrace()
    cable_trace.main()
