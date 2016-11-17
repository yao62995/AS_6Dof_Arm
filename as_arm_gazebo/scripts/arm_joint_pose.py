#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class ArmJointPose:
    def __init__(self):
        self.joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_flex_joint', 'wrist_rot_joint',
                            'finger_joint1', 'finger_joint2']
        # subscriber for setting joints position
        self.states_sub = rospy.Subscriber("/as_arm/set_joints_states", JointState, self.callback,
                                           queue_size=2)
        # a list of publisher
        self.joint_pub = dict()
        self.joint_pose = dict()
        for idx, name in enumerate(self.joint_names):
            pub = rospy.Publisher("/as_arm/joint%d_position_controller/command" % (idx + 1), Float64, queue_size=3)
            self.joint_pub[name] = pub
            self.joint_pose[name] = Float64()

    def callback(self, data):
        try:
            for name, pos in zip(data.name, data.position):
                self.joint_pose[name].data = pos
                self.joint_pub[name].publish(self.joint_pose[name])
        except ValueError:
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('arm_joint_pose', anonymous=True)
        gp = ArmJointPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
