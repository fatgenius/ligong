#!/usr/bin/env python
#coding=utf-8
import rospy
import os
from std_msgs.msg import String
import xml.etree.ElementTree as ET
from geometry_msgs.msg import PoseStamped

#回调函数输入的应该是msg
def GoalrevCallback(goalName):

    for tag in root:
        if(tag.attrib["name"]==goalName.data):
            goal = PoseStamped()
            rospy.loginfo("get goal:%s", goalName.data)
            goal.header.frame_id = tag.find("header").find("frame_id").text
            goal.pose.position.x = float(tag.find("pose").find("position").find("x").text)
            goal.pose.position.y = float(tag.find("pose").find("position").find("y").text)
            goal.pose.position.z = float(tag.find("pose").find("position").find("z").text)

            goal.pose.orientation.x = float(tag.find("pose").find("orientation").find("x").text)
            goal.pose.orientation.y = float(tag.find("pose").find("orientation").find("y").text)
            goal.pose.orientation.w = float(tag.find("pose").find("orientation").find("w").text)
            goal.pose.orientation.z = float(tag.find("pose").find("orientation").find("z").text)
            goalPub.publish(goal)
        else:
            rospy.loginfo('no such goal name:%s', goalName.data)



if __name__ == '__main__':
    path = os.path.split(os.path.realpath(__file__))[0]
    tree = ET.parse(path+'/goal.xml')
    root = tree.getroot()

    rospy.init_node('goalpub', anonymous=True)
    goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped , queue_size=1)
    rospy.Subscriber('/goal_name', String, GoalrevCallback)

    rospy.spin()