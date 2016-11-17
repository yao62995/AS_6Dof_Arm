#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: yao62995<yao_62995@163.com>

import rospy
from std_msgs.msg import UInt16MultiArray

import serv

import config

control = serv.Control(port=config.TTY_PROT)
joints = UInt16MultiArray()


def talker():
    pub = rospy.Publisher('servo_read', UInt16MultiArray, queue_size=10)
    rospy.init_node('servo_reader', anonymous=True)

    # pub = rospy.Publisher('gripper_read', JointStatus, queue_size=10)
    # rospy.init_node('gripper_reader', anonymous=True)
    rate = rospy.Rate(1)  #
    while not rospy.is_shutdown():
        joints.data = control.read_joint()
        pub.publish(joints)
        rate.sleep()

if __name__ == '__main__':
    talker()
