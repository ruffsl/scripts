#!/usr/bin/env python
from __future__ import print_function

import actionlib
import rospy

from control_msgs.msg import PointHeadAction, PointHeadGoal

from geometry_msgs.msg import PointStamped

from people_msgs.msg import PositionMeasurementArray, PositionMeasurement
from tf import TransformListener

class Follower():

    def __init__(self, *args):
        self.tf = TransformListener()
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        self.client.wait_for_server()
        rospy.Subscriber("/face_detector/people_tracker_measurements_array", PositionMeasurementArray, self.callback)
        self.rate = rospy.Rate(30)
        self.old_object_id = None
        rospy.spin()

    def callback(self, msg):
        for person in msg.people:
            if person.object_id == self.old_object_id:
                break
        point = PointStamped()
        point.header = person.header
        point.point = person.pos
        point_rf = self.tf.transformPoint("base_link", point)
        position = (point_rf.point.x, point_rf.point.y, point_rf.point.z)
        self.set_point_head(position)
        self.old_object_id = person.object_id
        self.rate.sleep()


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

if __name__ == '__main__':
    rospy.init_node('looker')
    try:
        ne = Follower()
    except rospy.ROSInterruptException:
        pass
