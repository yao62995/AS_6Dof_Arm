import rospy
from std_msgs.msg import UInt16MultiArray

joints = UInt16MultiArray()


def talker():
    pub = rospy.Publisher('servo_write', UInt16MultiArray, queue_size=10)
    rospy.init_node('servo_test', anonymous=True)
    rate = rospy.Rate(1000)  # 10hz
    while not rospy.is_shutdown():
        joints.mode = 1
        joints.angles = [0, 0, 0, 2, 0, 0]
        pub.publish(joints)
        rate.sleep()


if __name__ == '__main__':
    talker()
