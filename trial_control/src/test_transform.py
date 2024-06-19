#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs

rospy.init_node('tf_listener')

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Wait for the transform to become available
try:
    transform = tf_buffer.lookup_transform('wrist_3_link', 'tool0', rospy.Time(0), rospy.Duration(1.0))
    print("Transformation between wrist_3_link and tool0:")
    print(transform)
except tf2_ros.LookupException as ex:
    rospy.logerr('Transform lookup failed: %s', ex)

rospy.spin()
