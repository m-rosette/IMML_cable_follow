#!/usr/bin/env python3

import rospy
from trial_control.srv import CablePullTrigger, CablePullTriggerResponse

class CablePullTriggerServer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('cable_pull_trigger_server')
        rospy.loginfo("Starting cable pull trigger service server")
        
        # Create the service server
        self.service = rospy.Service('cable_pull_trigger', CablePullTrigger, self.handle_cable_pull_trigger)
        
        rospy.loginfo("Cable pull trigger service server ready")

    def handle_cable_pull_trigger(self, req):
        rospy.loginfo("Cable pull trigger service called")
        # Implement your logic here
        success = True  # Assume the operation is successful
        return CablePullTriggerResponse(success)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    server = CablePullTriggerServer()
    server.spin()
