#!/usr/bin/env python
from __future__ import print_function

import actionlib
import rospy

from control_msgs.msg import PointHeadAction, PointHeadGoal
from tf import TransformListener


class Follower():
    
    def __init__(self, *args):
        self.tf = TransformListener()
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        self.client.wait_for_server()
        
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            position = self.get_tf_position("/base_link", "/gripper_link")
            self.set_point_head(position)
            rate.sleep()
    
    def set_point_head(self, position):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = "base_link"
        goal.target.point.x = position[0]
        goal.target.point.y = position[1]
        goal.target.point.z = position[2]
        goal.min_duration = rospy.Duration(.5)
        goal.max_velocity = 1

        self.client.send_goal(goal)
#         self.client.wait_for_result()
#         return self.client.get_result()
        

    def get_tf_position(self, tf_a, tf_b):
        position = (0,0,0)
        if self.tf.frameExists(tf_a) and self.tf.frameExists(tf_b):
            self.tf.waitForTransform(tf_a, tf_b, rospy.Time(0), rospy.Duration(5))
            position, quaternion = self.tf.lookupTransform(tf_a, tf_b, rospy.Time(0))
#             print("Yes")
#         else:
#             print("Nope")
        return position


if __name__ == '__main__':
    rospy.init_node('follower')
    try:
        ne = Follower()
    except rospy.ROSInterruptException:
        pass
