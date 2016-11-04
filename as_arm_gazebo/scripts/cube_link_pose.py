#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates


class CubesLinkPose:
    def __init__(self):
        self.cubes_index = []
        self.cubes_pose = LinkStates()

        self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        self.pose_pub = rospy.Publisher("/gazebo/cubes", LinkStates, queue_size=1)

    def parse_cubes(self, links):
        for idx, link in enumerate(links):
            if link.startswith('cubes::'):
                self.cubes_index.append(idx)
                cubes_name = link[7:]
                self.cubes_pose.name.append(cubes_name)
        self.cubes_pose.pose = [0] * len(self.cubes_index)

    def callback(self, data):
        try:
            if len(self.cubes_index) == 0:
                self.parse_cubes(data.name)
            for i, c_index in enumerate(self.cubes_index):
                self.cubes_pose.pose[i] = data.pose[c_index]
        except ValueError:
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('cube_link_pose', anonymous=True)
        gp = CubesLinkPose()
        publish_rate = rospy.get_param('~publish_rate', 10)

        rate = rospy.Rate(publish_rate)
        while not rospy.is_shutdown():
            gp.pose_pub.publish(gp.cubes_pose)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
