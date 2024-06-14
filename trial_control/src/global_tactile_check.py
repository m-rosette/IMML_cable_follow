#!/usr/bin/env python3

import rospy
from trial_control.srv import TactileGlobal, TactileGlobalResponse
from papillarray_ros_v2.msg import SensorState


class TactileGlobalCheck:
    def __init__(self):
        rospy.init_node('tactile_global_check')
        rospy.loginfo("Starting tactile global check")

        self.tactile_0_global = []
        self.tactile_1_global = []

        # Subscribe to tactile sensor feedback
        self.tactile_0_sub = rospy.Subscriber('hub_0/sensor_0', SensorState, self.tactile_0_callback)
        self.tactile_1_sub = rospy.Subscriber('hub_0/sensor_1', SensorState, self.tactile_1_callback)

        # Create service to get the tactile global forces/torques
        self.tactile_global_srv = rospy.Service('tactile_global', TactileGlobal, self.tactile_global_callback)

    def tactile_0_callback(self, tac_msg):
        self.tactile_0_global = [tac_msg.gfX, tac_msg.gfY, tac_msg.gfZ]

    def tactile_1_callback(self, tac_msg):
        self.tactile_1_global = [tac_msg.gfX, tac_msg.gfY, tac_msg.gfZ]

    def tactile_global_callback(self, request):
        rospy.loginfo('Received global threshold request')
        response = TactileGlobalResponse()

        try:
            # Update response
            response.tactile0_global = self.tactile_0_global
            response.tactile1_global = self.tactile_1_global

        except ValueError:
            pass

        return response


def main():
    tactile_global_check = TactileGlobalCheck()
    rospy.spin()


if __name__ == '__main__':
    main()
