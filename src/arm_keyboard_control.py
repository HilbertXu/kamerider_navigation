#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/03/07
    Author: Xu Yucheng
    Abstract: control arm with keyboard
"""

import os
import sys
import time
import requests
import json
import hashlib
import base64
import rospy
import sys, select, termios, tty
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float64
from std_msgs.msg import UInt32

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class arm_keyboard():
    def __init__(self):
        self.joint1 = rospy.Publisher('/arm_shoulder_pan_joint/command',Float64)
        self.joint2 = rospy.Publisher('/arm_shoulder_lift_joint/command',Float64)
        self.joint3 = rospy.Publisher('/arm_elbow_flex_joint/command',Float64)
        self.joint4 = rospy.Publisher('/arm_wrist_flex_joint/command',Float64)
        self.joint5 = rospy.Publisher('/gripper_joint/command',Float64)
        self.msg = """
                    Control Your Turtlebot arm!
                    ---------------------------
                    Moving around:
                    t    y    u    i    o
                    g    h    j    k    l
                    CTRL-C to quit
                    """
        print (self.msg)
        self.keyboard_control()
    def keyboard_control(self):
        while(1):
            key = getKey()
            if key == 't':
                self.joint1.publish(0.2)
            if key == 'g':
                self.joint1.publish(-0.2)
            if key == 'y':
                self.joint2.publish(0.2)
            if key == 'h':
                self.joint2.publish(-0.2)
            if key == 'u':
                self.joint3.publish(0.2)
            if key == 'j':
                self.joint3.publish(-0.2)
            if key == 'i':
                self.joint4.publish(0.2)
            if key == 'k':
                self.joint4.publish(-0.2)
            if key == 'o':
                self.joint5.publish(0.2)
            if key == 'l':
                self.joint5.publish(-0.2)
            
if __name__ == '__main__':
    rospy.init_node("arm_keyboard_control")
    ak = arm_keyboard()
    rospy.spin()        