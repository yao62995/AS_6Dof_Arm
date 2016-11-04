import sys
import signal
import copy
import rospy
import moveit_commander

from std_msgs.msg import Float32
from std_msgs.msg import UInt16MultiArray

is_exit = False
DEG_TO_RAD = 0.01745329251994329577  # PI/180

joint_status = UInt16MultiArray()
gripper_status = Float32()

joint_map = {'base_joint': {"i": 0, 'bias': 90},
             'shoulder_joint': {"i": 1, 'bias': 95},
             'elbow_joint': {"i": 2, 'bias': 90},
             'wrist_flex_joint': {"i": 3, 'bias': 90},
             'wrist_rot_joint': {"i": 4, 'bias': 90},
             'gripper_joint': {"i": 5, 'bias': 90}
             }


def gripper_talker(pub, plan_msg, rate):
    points = plan_msg.joint_trajectory.points
    for i in xrange(len(points)):
        gripper_pos = abs(points[i].positions[0])
        # publish joint
        gripper_status.data = gripper_pos
        pub.publish(gripper_status)
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
    joint_status.data = joint_bias
    pub.publish(joint_status)
    rospy.sleep(5)


def init_gripper(group, pub):
    print "=======close gripper"
    # init position of simulator
    close_gripper = {"finger_joint1": 0, "finger_joint2": 0}
    plan_msg = group.plan(joints=close_gripper)
    group.execute(plan_msg=plan_msg, wait=True)
    rospy.sleep(2)
    # init position of real
    gripper_status.data = 0
    pub.publish(gripper_status)
    rospy.sleep(2)


def run_gripper_driver():
    # init moveit commander
    moveit_commander.roscpp_initialize(sys.argv)
    # specify move group
    arm_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    # init publisher
    arm_pub = rospy.Publisher('servo_write', JointStatus, queue_size=1)
    gripper_pub = rospy.Publisher('gripper_write', Float32, queue_size=1)
    # init ros node
    rospy.init_node('real_servo_driver', anonymous=True)

    print "============ Waiting for RVIZ..."
    rospy.sleep(2)
    # move grasper to init position
    init_position(arm_group, arm_pub)
    init_gripper(gripper_group, gripper_pub)

    # set ros publisher rate, 10hz = 10 seconds for a circle
    rate = rospy.Rate(100)

    # set position
    close_gripper = {"finger_joint1": 0, "finger_joint2": 0}
    open_gripper = {"finger_joint1": 0.015, "finger_joint2": -0.015}
    switch = True
    while True:
        if switch:
            plan_msg = gripper_group.plan(joints=open_gripper)
        else:
            plan_msg = gripper_group.plan(joints=close_gripper)
        switch = not switch
        gripper_group.execute(plan_msg=plan_msg, wait=False)
        # servo talker
        gripper_talker(gripper_pub, plan_msg, rate)
        rospy.sleep(5)
        if is_exit:
            break
    # shutdown moveit commander
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
        run_gripper_driver()
    except rospy.ROSInterruptException:
        pass
