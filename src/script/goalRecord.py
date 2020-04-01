#!/usr/bin/env python
#coding=utf-8
import rospy
import os
import sys
import xml.etree.ElementTree as ET
from geometry_msgs.msg import PoseStamped

#回调函数输入的应该是msg
def callback(pose_now):
    global flag
    global path
    global line
    if flag == 0:
        return
    root = tree.getroot()

    goal = ET.Element("goal")
    goal.attrib = {"name": line}
    
    header = ET.Element("header")
    frame_id = ET.Element("frame_id")
    frame_id.text = '/'+str(pose_now.header.frame_id)
    pose = ET.Element("pose")
    position = ET.Element("position")
    orientation = ET.Element("orientation")

    px = ET.Element("x")
    py = ET.Element("y")
    pz = ET.Element("z")

    ox = ET.Element("x")
    oy = ET.Element("y")
    ow = ET.Element("w")
    oz = ET.Element("z")

    px.text = str(pose_now.pose.position.x)
    py.text = str(pose_now.pose.position.y)
    pz.text = str(pose_now.pose.position.z)

    ox.text = str(pose_now.pose.orientation.x)
    oy.text = str(pose_now.pose.orientation.y)
    ow.text = str(pose_now.pose.orientation.w)
    oz.text = str(pose_now.pose.orientation.z)

    position.insert(len(position.getchildren()), px)
    position.insert(len(position.getchildren()), py)
    position.insert(len(position.getchildren()), pz)

    orientation.insert(len(orientation.getchildren()), ox)
    orientation.insert(len(orientation.getchildren()), oy)
    orientation.insert(len(orientation.getchildren()), ow)
    orientation.insert(len(orientation.getchildren()), oz)

    header.insert(len(header.getchildren()), frame_id)
    pose.insert(len(pose.getchildren()), position)
    pose.insert(len(pose.getchildren()), orientation)


    goal.insert(len(goal.getchildren()), header)
    goal.insert(len(goal.getchildren()), pose)
    root.insert(len(root.getchildren()), goal)

    tree.write(path+'/goal.xml')
    flag = 0

if __name__ == '__main__':
    flag = 1
    path = os.path.split(os.path.realpath(__file__))[0]
    tree = ET.parse(path+'/goal.xml')
    rospy.init_node('pylistener', anonymous=True)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
    print "please type the goal name below:\n"
    while not rospy.is_shutdown():
        line = sys.stdin.readline()
        line = line.strip()
        rospy.spin()