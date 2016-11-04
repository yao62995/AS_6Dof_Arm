#!/usr/bin/env python

import sys
import signal
import rospy
import moveit_commander

from std_msgs.msg import UInt16MultiArray

joint_status = UInt16MultiArray()

is_exit = False
DEG_TO_RAD = 0.01745329251994329577  # PI/180

joint_map = {'base_joint': {"i": 0, 'bias': 90},
             'shoulder_joint': {"i": 1, 'bias': 95},
             'elbow_joint': {"i": 2, 'bias': 90},
             'wrist_flex_joint': {"i": 3, 'bias': 90},
             'wrist_rot_joint': {"i": 4, 'bias': 90},
             'gripper_joint': {"i": 5, 'bias': 90}
             }


def servo_talker(pub, plan_msg, rate):
    joint_names = plan_msg.joint_trajectory.joint_names
    points = plan_msg.joint_trajectory.points
    for i in xrange(len(points)):
        joint_angles = [0] * 6
        # transform angles
        for idx, angle in enumerate(points[i].positions):
            joint_param = joint_map[joint_names[idx]]
            joint_angles[joint_param["i"]] = int(angle / DEG_TO_RAD) + joint_param['bias']
        # ignore gripper_joint
        joint_angles[5] = 90
        # set joint status
        joint_status.data = joint_angles
        # publish joint
        pub.publish(joint_status)
        # wait according to hz
        # rate.sleep()
        rospy.sleep(0.1)
        if is_exit:
            sys.exit(0)


def init_position(group, pub):
    print "=======move to UpRight position"
    # init position of simulator
    up_right_joint_values = {"base_joint": 0, "shoulder_joint": 0, "elbow_joint": 0,
                             "wrist_flex_joint": 0, "wrist_rot_joint": 0}
    plan_msg = group.plan(joints=up_right_joint_values)
    group.execute(plan_msg=plan_msg, wait=True)
    rospy.sleep(2)
    # init position of real
    joint_bias = [0] * 6
    for _, v in joint_map.items():
        joint_bias[v["i"]] = v['bias']
    joint_status.mode = 0
    joint_status.angles = joint_bias
    pub.publish(joint_status)
    rospy.sleep(10)


def run_driver():
    # init moveit commander
    moveit_commander.roscpp_initialize(sys.argv)
    # specify move group
    group = moveit_commander.MoveGroupCommander("arm")
    # init publisher
    pub = rospy.Publisher('servo_write', JointStatus, queue_size=10)
    # init ros node
    rospy.init_node('real_servo_driver', anonymous=True)

    print "============ Waiting for RVIZ..."
    rospy.sleep(2)
    # move grasper to init position
    init_position(group, pub)

    # set ros publisher rate, 10hz = 10 seconds for a circle
    rate = rospy.Rate(100)

    # set position
    pre_grasp_joint_values = {"base_joint": -0.8295, "shoulder_joint": 0.5834, "elbow_joint": 0.9089,
                              "wrist_flex_joint": 1.006, "wrist_rot_joint": 0}
    test_joint_values = {"base_joint": 0, "shoulder_joint": 0, "elbow_joint": 0,
                         "wrist_flex_joint": 90 * DEG_TO_RAD, "wrist_rot_joint": 0}
    plan_msg = group.plan(joints=test_joint_values)
    group.execute(plan_msg=plan_msg, wait=False)
    # servo talker
    servo_talker(pub, plan_msg, rate)
    rospy.sleep(5)
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
