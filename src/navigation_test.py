#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import os
import sys


class navigation_test():
    def __init__(self):
        self.sp_pub = rospy.Publisher("/speech_pub", String, queue_size=1)
        rospy.Subscriber("/nav_pub", String, self.navCallback)
        self.main()

    def navCallback(self, msg):
        if msg.data.lower() == 'arrive_at_pos1':
            rospy.loginfo("Now Go to Position 2")
            self.sp_pub.publish("go_to_pos2")
        
        if msg.data.lower() == "arrive_at_pos2":
            rospy.loginfo("start grasping object")
            os.system("roslaunch turtlebot_arm_bringup myarm.launch")
            rospy.sleep(1)
            os.system("rosrun turtlebot_arm_moveit_demos moveit_ik_demos.py")
            rospy.sleep(1)
    def main(self):
        rospy.sleep(2)
        rospy.loginfo("Go to Position 1")
        self.sp_pub.publish("go_to_pos1")

if __name__ == '__main__':
    rospy.init_node("navigation_test", anonymous=True)
    nt = navigation_test()
    rospy.spin()
