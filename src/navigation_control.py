#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import sys


class navigation_test():
    def __init__(self):
        self.sp_pub = rospy.Publisher("/speech_pub", String, queue_size=1)
        rospy.Subscriber("/nav_pub", String, self.navCallback)
        self.nav_pub = rospy.Publisher("/kamerider_navigation/nav_pub", String, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=1)
        self.main()

    def navCallback(self, msg):
        if msg.data.lower() == 'arrive_at_pos1':
            rospy.loginfo("Now Go to Position 2")
            self.sp_pub.publish("go_to_pos2")
        
        if msg.data.lower() == "arrive_at_pos2":
            rospy.loginfo("start grasping object")
            msg = Twist()
            rate = rospy.Rate(1)
            msg.linear.x = 0.1
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            for i in range(3):
                self.cmd_pub.publish(msg)
                rate.sleep()
            os.system("roslaunch darknet_ros darknet_ros.launch")
            rospy.sleep(5)
            os.system("rosrun kamerider_image_detection object_detection")
            rospy.sleep(1)
            self.nav_pub.publish("in_grasp_position")
            rospy.sleep(1)
            os.system("rosrun turtlebot_arm_moveit_demos grasp_target_object.py")
            rospy.sleep(1)
    def main(self):
        rospy.sleep(2)
        rospy.loginfo("Go to Position 1")
        self.sp_pub.publish("go_to_pos1")

if __name__ == '__main__':
    rospy.init_node("navigation_test", anonymous=True)
    nt = navigation_test()
    rospy.spin()
