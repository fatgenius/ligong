#!/usr/bin/env python
# coding=utf-8
import rospy
import sys
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('goal_name_pub', anonymous=True)
    Namepub = rospy.Publisher('/goal_name', String, queue_size=1)

    while not rospy.is_shutdown():
        print "please type the goal name below:\n"
        line = sys.stdin.readline()
        line = line.strip()
        name = line
        Namepub.publish(name)
