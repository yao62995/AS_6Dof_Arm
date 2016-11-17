#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: yao62995<yao_62995@163.com>

import rospy
from std_msgs.msg import UInt16

import serv

import config

control = serv.Control(port=config.TTY_PROT)


# 0~1, 0 is closed, 1 is open
def callback_gripper(data):
    degree = float(data.data) / 0.015
    control.write_head_status(degree, num=5)


def listener():

    rospy.init_node('gripper_listener', anonymous=True)
    rospy.Subscriber('gripper_write', UInt16, callback_gripper)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
