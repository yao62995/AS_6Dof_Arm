#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: yao62995<yao_62995@163.com>

import sys
import signal
import copy
import rospy
import moveit_commander

#  END_SUB_TUTORIAL

is_exit = False
DEG_TO_RAD = 0.01745329251994329577  # PI/180


def test_grasp_planer():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('as_arm_move_group', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("gripper")
    print "============ Waiting for RVIZ..."
    rospy.sleep(2)
    print "============ Starting tutorial "
    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ Reference frame: %s" % group.get_end_effector_link()
    print "============ Robot Groups: %s" % ", ".join(robot.get_group_names())
    print "============ Printing robot state"
    print robot.get_current_state()
    switch = True
    while True:
        print "============ Generating plan 1"
        # =============== specify target config ==============
        close_gripper = {"finger_joint1": 0, "finger_joint2": 0}
        open_gripper = {"finger_joint1": 0.015, "finger_joint2": -0.015}
        if switch:
            plan_msg = group.plan(joints=close_gripper)
        else:
            plan_msg = group.plan(joints=open_gripper)
        switch = not switch
        group.execute(plan_msg=plan_msg, wait=True)
        rospy.sleep(5)

        if is_exit:
            break
    moveit_commander.roscpp_shutdown()


def transform_plan_msgs(plan_msg):
    joint_names = plan_msg.joint_trajectory.joint_names
    points = plan_msg.joint_trajectory.points
    joint_traj = []
    joint_bias = [90, 95, 150, 90, 90, 90]
    for i in xrange(len(points)):
        # all angles is absolute angles
        joint_angles = points[i].positions
        # transform angles
        for idx, angle in enumerate(joint_angles):
            joint_angles[idx] = int(angle / DEG_TO_RAD) + joint_bias[idx]
        joint_traj.append(joint_angles)
    return joint_names, joint_traj


def signal_handler(signum, frame):
    global is_exit
    is_exit = True
    print "receive a signal %d, is_exit = %d" % (signum, is_exit)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    try:
        # move_group_python_interface_tutorial()
        test_grasp_planer()
    except rospy.ROSInterruptException:
        pass