#!/usr/bin/env python

import sys
import signal
import rospy
import moveit_commander


is_exit = False
DEG_TO_RAD = 0.01745329251994329577  # PI/180

joint_map = {'base_joint': {"i": 0, 'bias': 90},
             'shoulder_joint': {"i": 1, 'bias': 95},
             'elbow_joint': {"i": 2, 'bias': 90},
             'wrist_flex_joint': {"i": 3, 'bias': 90},
             'wrist_rot_joint': {"i": 4, 'bias': 90},
             'gripper_joint': {"i": 5, 'bias': 90}
             }


def init_position(group):
    # init position of simulator
    up_right_joint_values = {"base_joint": 0, "shoulder_joint": 0, "elbow_joint": 0,
                             "wrist_flex_joint": 0, "wrist_rot_joint": 0}
    plan_msg = group.plan(joints=up_right_joint_values)
    group.execute(plan_msg=plan_msg, wait=True)
    rospy.sleep(2)


def run_driver():
    # init moveit commander
    moveit_commander.roscpp_initialize(sys.argv)
    # specify move group
    group = moveit_commander.MoveGroupCommander("arm")
    # init ros node
    rospy.init_node('real_servo_driver', anonymous=True)

    print "============ Waiting for RVIZ..."
    rospy.sleep(2)
    # move grasper to init position
    init_position(group)

    # set ros publisher rate, 10hz = 10 seconds for a circle
    rate = rospy.Rate(50)
    while True:
        group.set_random_target()
        plan_msg = group.plan()
        group.execute(plan_msg=plan_msg, wait=False)
        rospy.sleep(5)
        if is_exit:
            break
    # shutdown moveit commander
    moveit_commander.roscpp_shutdown()


def signal_handler(signum, frame):
    global is_exit
    is_exit = True
    print "receive a signal %d, is_exit = %d" % (signum, is_exit)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    try:
        # move_group_python_interface_tutorial()
        run_driver()
    except rospy.ROSInterruptException:
        pass
