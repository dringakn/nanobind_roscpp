#!/usr/bin/env python3
import rospy
from nanobind_roscpp.point_utils_py import Point, distance

def talker():
    rospy.init_node('distance_node')
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        a = Point(0, 0)
        b = Point(1, 1)
        rospy.loginfo(f"Dist: {distance(a, b)}")
        r.sleep()

if __name__ == '__main__':
    talker()
