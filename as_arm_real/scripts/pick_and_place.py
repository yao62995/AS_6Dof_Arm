#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: yao62995<yao_62995@163.com>

import sys
import signal
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float32


class PickAndPlace(object):
    DEG_TO_RAD = 0.01745329251994329577  # PI/180

    joint_map = {'base_joint': {"i": 0, 'bias': 90},
                 'shoulder_joint': {"i": 1, 'bias': 95},
                 'elbow_joint': {"i": 2, 'bias': 90},
                 'wrist_flex_joint': {"i": 3, 'bias': 90},
                 'wrist_rot_joint': {"i": 4, 'bias': 90},
                 'gripper_joint': {"i": 5, 'bias': 90}
                 }

    close_gripper_conf = {"finger_joint1": 0, "finger_joint2": 0}
    open_gripper_conf = {"finger_joint1": 0.015, "finger_joint2": -0.015}

    def __init__(self):
        self.joint_status = UInt16MultiArray()
        self.gripper_status = Float32()

        self.is_exit = False
        self.run_real = False

        # init moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        # specify move group
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        # init publisher
        self.arm_pub = rospy.Publisher('servo_write', UInt16MultiArray, queue_size=10)
        self.gripper_pub = rospy.Publisher('gripper_write', Float32, queue_size=10)
        # init ros node
        rospy.init_node('real_servo_driver', anonymous=True)
        # init robot and scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # set ros publisher rate, 10hz = 10 seconds for a circle
        self.rate = rospy.Rate(50)

        print "============ Waiting for RVIZ..."
        rospy.sleep(2)
        # move grasper to init position
        self.init_position()
        self.init_gripper()

    def init_position(self):
        print "=======init: reset position"
        # init position of simulator
        up_right_joint_values = {"base_joint": 0, "shoulder_joint": 0, "elbow_joint": 0,
                                 "wrist_flex_joint": 0, "wrist_rot_joint": 0}
        plan_msg = self.arm_group.plan(joints=up_right_joint_values)
        self.arm_group.execute(plan_msg=plan_msg, wait=False)
        self.servo_talker(plan_msg)
        rospy.sleep(5)

    def init_gripper(self):
        print "=======init: close gripper"
        self.close_gripper()
        rospy.sleep(5)

    def open_gripper(self):
        plan_msg = self.gripper_group.plan(joints=self.open_gripper_conf)
        self.gripper_group.execute(plan_msg=plan_msg, wait=False)
        self.gripper_talker(plan_msg)

    def close_gripper(self):
        plan_msg = self.gripper_group.plan(joints=self.close_gripper_conf)
        self.gripper_group.execute(plan_msg=plan_msg, wait=False)
        self.gripper_talker(plan_msg)

    def pickup(self, pos):
        # open gripper
        print "=======pickup: open gripper"
        self.open_gripper()
        rospy.sleep(5)
        # move to pos
        print "=======pickup: move gripper"
        self.arm_group.set_pose_target(pos)
        plan_msg = self.arm_group.plan()
        self.arm_group.execute(plan_msg=plan_msg, wait=False)
        self.servo_talker(plan_msg)
        rospy.sleep(10)
        # close gripper
        print "=======init: close gripper"
        self.close_gripper()
        rospy.sleep(5)

    def place(self, pos):
        # move to pos
        print "=======place: move gripper"
        self.arm_group.set_pose_target(pos)
        plan_msg = self.arm_group.plan()
        self.arm_group.execute(plan_msg=plan_msg, wait=False)
        self.servo_talker(plan_msg)
        rospy.sleep(10)
        # open gripper
        print "=======init: open gripper"
        self.open_gripper()
        rospy.sleep(5)

    def create_cube(self, name, pos):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = geometry_msgs.msg.Quaternion(*q)

        # Coke can size from ~/.gazebo/models/coke_can/meshes/coke_can.dae,
        # using the measure tape tool from meshlab.
        # The box is the bounding box of the coke cylinder.
        # The values are taken from the cylinder base diameter and height.
        # self.scene.add_box(name, p, (0.01, 0.01, 0.01))

        return p.pose

    def servo_talker(self, plan_msg):
        if not self.run_real:
            return
        joint_names = plan_msg.joint_trajectory.joint_names
        points = plan_msg.joint_trajectory.points
        for i in xrange(len(points)):
            joint_angles = [0] * 6
            # transform angles
            for idx, angle in enumerate(points[i].positions):
                joint_param = self.joint_map[joint_names[idx]]
                joint_angles[joint_param["i"]] = int(angle / self.DEG_TO_RAD) + joint_param['bias']
            # ignore gripper_joint
            joint_angles[5] = 90
            # set joint status
            self.joint_status.data = joint_angles
            # publish joint
            self.arm_pub.publish(self.joint_status)
            # wait according to hz
            # self.rate.sleep()
            rospy.sleep(0.1)
            if self.is_exit:
                sys.exit(0)

    def gripper_talker(self, plan_msg):
        if not self.run_real:
            return
        points = plan_msg.joint_trajectory.points
        for i in xrange(len(points)):
            gripper_pos = abs(points[i].positions[0])
            # publish joint
            self.gripper_status.data = gripper_pos
            self.gripper_pub.publish(self.gripper_status)
            # wait according to hz
            # self.rate.sleep()
            rospy.sleep(0.1)
            if self.is_exit:
                sys.exit(0)

    def create_pose(self, _pos, _orient=[0, 0, 0], name="cube"):
        ppose = self.arm_group.get_random_pose()
        pose = geometry_msgs.msg.Pose()
        pose.position.x = _pos[0]
        pose.position.y = _pos[1]
        pose.position.z = _pos[2]
        # q = quaternion_from_euler(_orient[0], _orient[1], _orient[2])
        # pose.orientation = geometry_msgs.msg.Quaternion(*q)
        pose.orientation.x = _orient[0]
        pose.orientation.y = _orient[1]
        pose.orientation.z = _orient[2]
        pose.orientation.w = _orient[3]
        ppose.pose = pose
        # self.scene.add_sphere(name, ppose, radius=0.01)
        return pose


def pick_and_place():
    handler = PickAndPlace()
    handler.run_real = True
    source_pose = handler.create_pose([-0.0176728656253, -0.0384421037616, 0.224307380251],
                                      [0.457132535307, -0.486117624337, 0.405314466878, 0.624851729143],
                                      name="cube_1")
    target_pose = handler.create_pose([0.123779063019, 0.194509885849, 0.101373193118],
                                      [-0.525018058158, 0.310156939143, -0.0537275264307, 0.79074146509],
                                      name="cube_2")
    # pickup object
    # source_pose = handler.arm_group.get_random_pose()
    # target_pose = handler.arm_group.get_random_pose()
    print "===source pose:", source_pose
    print "===target pose:", target_pose
    handler.pickup(source_pose)
    # place object
    handler.place(target_pose)
    # return init position
    handler.init_position()
    handler.init_gripper()


def signal_handler(signum, frame):
    global is_exit
    is_exit = True
    print "receive a signal %d, is_exit = %d" % (signum, is_exit)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    try:
        # move_group_python_interface_tutorial()
        pick_and_place()
    except rospy.ROSInterruptException:
        pass
