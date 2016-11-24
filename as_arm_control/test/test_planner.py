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

#  END_SUB_TUTORIAL

is_exit = False
DEG_TO_RAD = 0.01745329251994329577  # PI/180

"""
================   robot state =====================
joint_state:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs:         0
    frame_id: /base_link
  name: ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_flex_joint', 'wrist_rot_joint', 'gripper_joint']
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  velocity: []
  effort: []
multi_dof_joint_state:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs:         0
    frame_id: /base_link
  joint_names: []
  transforms: []
  twist: []
  wrench: []
attached_collision_objects: []
is_diff: False




================= plan_msg: ========================
joint_trajectory:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs:         0
    frame_id: /base_link
  joint_names: ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_flex_joint', 'wrist_rot_joint']
  points:
    -
      positions: [0.5034020869194958, -0.8985533430451286, -0.5367264811050054, -0.9010551824488547, -0.005581752562049777]
      velocities: [0.21559119998688964, -0.384869624589417, -0.2298491350786716, -0.3858816255841634, -0.0023477902818256112]
      accelerations: [-0.5099329597095295, 0.9103233656156271, 0.5436569291532823, 0.9127170285411718, 0.005553174931358115]
      effort: []
      time_from_start:
        secs: 1
        nsecs: 638480175
    -
      positions: [0.5252887915240582, -0.9376251037903242, -0.5600606437395979, -0.9402296809998275, -0.005820098989802413]
      velocities: [0.13385741726502545, -0.23895991086107457, -0.1427099602585907, -0.23958824745104013, -0.001457708586548121]
      accelerations: [-0.5619232336094951, 1.003135489669845, 0.5990855342593152, 1.0057731987758647, 0.006119349523916666]
      effort: []
      time_from_start:
        secs: 1
        nsecs: 754089002
    -
      positions: [0.5471754961286206, -0.9766968645355198, -0.5833948063741904, -0.9794041795508005, -0.0060584454175550495]
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0]
      accelerations: [-0.561641101806141, 1.0026318329285258, 0.5987847439163739, 1.0052682176870553, 0.006116277105811696]
      effort: []
      time_from_start:
        secs: 2
        nsecs:  33263501
    .....


"""


def move_group_python_interface_tutorial():
    #  BEGIN_TUTORIAL
    # 
    #  Setup
    #  ^^^^^
    #  CALL_SUB_TUTORIAL imports
    # 
    #  First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    #  Instantiate a RobotCommander object.  This object is an interface to
    #  the robot as a whole.
    robot = moveit_commander.RobotCommander()

    #  Instantiate a PlanningSceneInterface object.  This object is an interface
    #  to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    #  Instantiate a MoveGroupCommander object.  This object is an interface
    #  to one group of joints.  In this case the group is the joints in the left
    #  arm.  This interface can be used to plan and execute motions on the left
    #  arm.
    group = moveit_commander.MoveGroupCommander("AS_Arm")

    #  We create this DisplayTrajectory publisher which is used below to publish
    #  trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    #  Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(5)
    print "============ Starting tutorial "

    #  Getting Basic Information
    #  ^^^^^^^^^^^^^^^^^^^^^^^^^
    # 
    #  We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % group.get_planning_frame()

    #  We can also print the name of the end-effector link for this group
    print "============ Reference frame: %s" % group.get_end_effector_link()

    #  We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()

    #  Sometimes for debugging it is useful to print the entire state of the
    #  robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    #  Planning to a Pose goal
    #  ^^^^^^^^^^^^^^^^^^^^^^^
    #  We can plan a motion for this group to a desired pose for the
    #  end-effector
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.7
    pose_target.position.y = -0.05
    pose_target.position.z = 1.1
    group.set_pose_target(pose_target)

    #  Now, we call the planner to compute the plan
    #  and visualize it if successful
    #  Note that we are just planning, not asking move_group
    #  to actually move the robot
    plan1 = group.plan()

    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)

    #  You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
    #  group.plan() method does this automatically so this is not that useful
    #  here (it just displays the same trajectory again).
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory)

    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(5)

    #  Moving to a pose goal
    #  ^^^^^^^^^^^^^^^^^^^^^
    #
    #  Moving to a pose goal is similar to the step above
    #  except we now use the go() function. Note that
    #  the pose goal we had set earlier is still active
    #  and so the robot will try to move to that goal. We will
    #  not use that function in this tutorial since it is
    #  a blocking function and requires a controller to be active
    #  and report success on execution of a trajectory.

    # Uncomment below line when working with a real robot
    # group.go(wait=True)

    #  Planning to a joint-space goal
    #  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #
    #  Let's set a joint space goal and move towards it.
    #  First, we will clear the pose target we had just set.

    group.clear_pose_targets()

    #  Then, we will get the current set of joint values for the group
    group_variable_values = group.get_current_joint_values()
    print "============ Joint values: ", group_variable_values

    #  Now, let's modify one of the joints, plan to the new joint
    #  space goal and visualize the plan
    group_variable_values[0] = 1.0
    group.set_joint_value_target(group_variable_values)

    plan2 = group.plan()

    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(5)

    #  Cartesian Paths
    #  ^^^^^^^^^^^^^^^
    #  You can plan a cartesian path directly by specifying a list of waypoints
    #  for the end-effector to go through.
    waypoints = []

    # start with the current pose
    waypoints.append(group.get_current_pose().pose)

    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x + 0.1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

    # second move down
    wpose.position.z -= 0.10
    waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))

    #  We want the cartesian path to be interpolated at a resolution of 1 cm
    #  which is why we will specify 0.01 as the eef_step in cartesian
    #  translation.  We will specify the jump threshold as 0.0, effectively
    #  disabling it.
    (plan3, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.01,  # eef_step
        0.0)  # jump_threshold

    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(5)

    #  Adding/Removing Objects and Attaching/Detaching Objects
    #  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #  First, we will define the collision object message
    collision_object = moveit_msgs.msg.CollisionObject()

    #  When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    #  END_TUTORIAL

    print "============ STOPPING"


def test_move_planer():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('as_arm_move_group', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")
    print "============ Waiting for RVIZ..."
    rospy.sleep(2)
    print "============ Starting tutorial "
    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ Reference frame: %s" % group.get_end_effector_link()
    print "============ Robot Groups: %s" % ", ".join(robot.get_group_names())
    print "============ Printing robot state"
    print robot.get_current_state()
    switch = True
    joint = group.get_random_joint_values()
    while True:
        print "============ Generating plan 1"
        # ========== random target pos =================
        # group.set_random_target()
        # plan_msg = group.plan()
        # group.execute(plan_msg=plan_msg, wait=True)
        # group.go(joints=joint, wait=True)


        # =============== specify target config ==============
        joint_names = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_flex_joint", "wrist_rot_joint"]
        up_right_joint_values = {"base_joint": 0, "shoulder_joint": 0, "elbow_joint": 0,
                                 "wrist_flex_joint": 0, "wrist_rot_joint": 0}
        pre_grasp_joint_values = {"base_joint": 0.5471, "shoulder_joint": -0.9766, "elbow_joint": -0.5834,
                                  "wrist_flex_joint": -0.9795, "wrist_rot_joint": -0.006}
        test_joint_values = [0, 0.65, -0.9736, -1.4648, 0]
        test_joint_map = dict((n, j) for n, j in zip(joint_names, test_joint_values))
        test_joint_values_2 = [1.5707, 0.5167, -1.0471, -1.456, 0]
        test_joint_map_2 = dict((n, j) for n, j in zip(joint_names, test_joint_values_2))
        if switch:
            plan_msg = group.plan(joints=test_joint_map)
        else:
            plan_msg = group.plan(joints=test_joint_map_2)
        switch = not switch
        group.execute(plan_msg=plan_msg, wait=True)
        rospy.sleep(3)

        # ============ specify target positions and orientations ==================
        """
header:
  seq: 0
  stamp:
    secs: 1477622358
    nsecs: 570399999
  frame_id: /base_link
pose:
  position:
    x: 0.0704592248086
    y: -0.259683593557
    z: 0.0291556146529
  orientation:
    x: 0.125357993591
    y: 0.62901329758
    z: -0.696759416612
    w: 0.321175902385

        """
        # end_effector = group.get_end_effector_link()
        # # pose_target = geometry_msgs.msg.Pose()
        # # pose_target.orientation.w = 0.321175902385
        # # pose_target.orientation.x = 0.125357993591
        # # pose_target.orientation.y = 0.62901329758
        # # pose_target.orientation.z = -0.696759416612
        # # pose_target.position.x = 0.0704592248086
        # # pose_target.position.y = -0.259683593557
        # # pose_target.position.z = 0.0291556146529
        # print
        # pose_target = geometry_msgs.msg.Pose()
        # pose_target.pose.position.x = -0.2
        # pose_target.pose.position.y = 0
        # pose_target.pose.position.z = 0.1
        # group.set_pose_target(pose_target)
        # # group.set_pose_target(group.get_random_pose())
        # plan_msg = group.plan()
        # group.execute(plan_msg=plan_msg, wait=True)
        # is_exit = True

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
        test_move_planer()
    except rospy.ROSInterruptException:
        pass
