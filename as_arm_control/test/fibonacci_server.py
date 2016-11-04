#! /usr/bin/env python

import roslib;

roslib.load_manifest('actionlib_tutorials')
import rospy

import actionlib

import actionlib_tutorials.msg


class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(4)
        success = True

        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        self._result.sequence = []

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (
            self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

        # start executing the action
        for i in xrange(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._result.sequence.append(self._feedback.sequence[1] + self._feedback.sequence[0])
            self._as.publish_feedback(self._feedback)
            # publish the feedback
            self._feedback.sequence[0] = self._feedback.sequence[1]
            self._feedback.sequence[1] = self._result.sequence[-1]

            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('fibonacci')
    FibonacciAction(rospy.get_name())
    rospy.spin()
