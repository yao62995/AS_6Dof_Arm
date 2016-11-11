#!/usr/bin/env python
import rospy
import tf


class TransformListener(object):
    def __init__(self, source_frame, target_frame):
        self.source_frame = source_frame
        self.target_frame = target_frame
        rospy.loginfo("transfrom listener: %s -> %s" % (source_frame, target_frame))
        self.listener = tf.TransformListener()

    def listern(self):
        (trans, rot) = self.listener.lookupTransform(self.source_frame, self.target_frame, rospy.Time(0))
        print trans, rot


if __name__ == '__main__':
    try:
        rospy.init_node('transfrom_demo', anonymous=True)
        gp = TransformListener("/world", "/gripper_bottom_link")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                gp.listern()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rospy.sleep(2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
