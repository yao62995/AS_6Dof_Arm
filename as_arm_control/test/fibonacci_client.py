#! /usr/bin/env python

import roslib;

roslib.load_manifest('actionlib_tutorials')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg


def feedback_cb(feed_back):
    print feed_back


def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_cb)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
