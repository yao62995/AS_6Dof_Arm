# !/usr/bin/env python

import sys
import signal
import rospy
import moveit_commander
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import Constraints, AttachedCollisionObject

is_exit = False


def test_check_collision():
    global is_exit
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('as_arm_check_collision', anonymous=True)

    robot = moveit_commander.RobotCommander()
    # arm_group = moveit_commander.MoveGroupCommander("arm")
    scene = moveit_commander.PlanningSceneInterface()

    print "==========links:", robot.get_link_names()

    # scene.attach_box("camera_mast_link", "camera_mast_link")
    check_state_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

    while not is_exit:
        robot_state = robot.get_current_state()

        col_obj = AttachedCollisionObject()
        col_obj.link_name = "camera_mast_link"
        robot_state.attached_collision_objects.append(col_obj)
        robot_state.is_diff = True

        print robot_state
        group_name = ""
        contraint = Constraints()
        resp = check_state_validity(robot_state, group_name, contraint)
        print "==================="
        print resp
        if not resp.valid:
            print "not valid"
            is_exit = True
        rospy.sleep(0.5)
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
        test_check_collision()
    except rospy.ROSInterruptException:
        pass
