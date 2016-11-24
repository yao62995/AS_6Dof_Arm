import sys
import thread
import rospy
import tf
import moveit_commander
from gazebo_msgs.msg import LinkStates, LinkState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from sensor_msgs.msg import CompressedImage, JointState

# init moveit commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('as_arm_show_pickup', anonymous=True)
# init robot model
ros_robot = moveit_commander.RobotCommander()
# init robot scene
ros_scene = moveit_commander.PlanningSceneInterface()
# init moveit group
arm_group = ros_robot.get_group("arm")


class CubesManager(object):
    def __init__(self):
        """
        :param cubes_name: a list of string type of all cubes
        """
        self.cubes_pose = LinkState()
        self.cubes_state = dict()
        # pos publisher
        self.pose_pub = rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=1)
        self.pose_sub = rospy.Subscriber("/gazebo/cubes", LinkStates, callback=self.callback_state, queue_size=1)

    def callback_state(self, data):
        for idx, link in enumerate(data.name):
            self.cubes_state.setdefault(link, [0] * 3)
            pose = self.cubes_state[link]
            pose[0] = data.pose[idx].position.x - 0.18
            pose[1] = data.pose[idx].position.y
            pose[2] = data.pose[idx].position.z + 0.05

    def read_cube_pose(self, name=None):
        if name is not None:
            return self.cubes_state[name]
        else:
            return self.cubes_state

    def set_cube_pose(self, name, pose, orient=None):
        """
        :param name: cube name, a string
        :param pose: cube position, a list of three float, [x, y, z]
        :param orient: cube orientation, a list of three float, [ix, iy, iz]
        :return:
        """
        self.cubes_pose.link_name = "cubes::" + name
        p = self.cubes_pose.pose
        p.position.x = pose[0] + 0.18
        p.position.y = pose[1]
        p.position.z = pose[2] - 0.05
        if orient is None:
            orient = [0, 0, 0]
        q = quaternion_from_euler(orient[0], orient[1], orient[2])
        p.orientation = Quaternion(*q)
        self.pose_pub.publish(self.cubes_pose)


# create cube
attach_flag = False
cube_mgr = CubesManager()
joint_state = JointState()
tf_listener = tf.TransformListener()
joint_pub = rospy.Publisher("/as_arm/set_joints_states", JointState, queue_size=1)
joint_arm_names = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_flex_joint", "wrist_rot_joint"]
joint_finger_names = ['finger_joint1', 'finger_joint2']

{'wrist_flex_joint': -1.425475606655082, 'shoulder_joint': 0.7000869653119977, 'base_joint': 0.06940273470292257,
 'wrist_rot_joint': -0.20631109160481775, 'elbow_joint': -0.8229608422235993}


joint_init_pos = [0, 0, 0, 0, 0]
# joint_pick_pos = [0, 0.62, -0.9736, -1.4648, 0]
joint_middle_pos = [0, 0.2834, -0.9736, -1.4648, 0]
joint_place_pos = [1.5707, 0.2834, -1.0471, -1.456, 0]
cube_init_pos = [-0.1695, 0.0078, 0.0447]

def joint_pub_func(name, val):
    joint_state.name = name
    joint_state.position = val
    joint_pub.publish(joint_state)


def open_gripper():
    joint_pub_func(joint_finger_names, [0, 0])


def close_gripper():
    joint_pub_func(joint_finger_names, [-0.006, 0.006])


def attach_cube_thread(cube_name):
    global attach_flag
    while attach_flag:
        (trans, _) = tf_listener.lookupTransform("/world", "/grasp_frame_link", rospy.Time(0))
        cube_mgr.set_cube_pose(cube_name, list(trans))


def move_plannar(source_pose, target_pose):
    source_joints = dict((n, j) for n, j in zip(joint_arm_names, source_pose))
    target_joints = dict((n, j) for n, j in zip(joint_arm_names, target_pose))
    arm_group.go(joints=source_pose)
    joint_poses = []
    while len(joint_poses) == 0:
        try:
            arm_group.go(joints=source_joints, wait=True)
            plan_msg = arm_group.plan(joints=target_joints)
            for p in plan_msg.joint_trajectory.points:
                joint_poses.append(p.positions)
        except:
            joint_poses = []
    return joint_poses


def move_handle(source_pos, target_pos):
    poses = move_plannar(source_pos, target_pos)
    for pos in poses:
        joint_pub_func(joint_arm_names, pos)
        rospy.sleep(0.3)
    rospy.sleep(3.0)


while True:
    # init
    open_gripper()
    rospy.sleep(2.0)
    joint_pub_func(joint_arm_names, joint_init_pos)
    cube_mgr.set_cube_pose("cube1", cube_init_pos)
    rospy.sleep(5.0)
    # pick
    move_handle(joint_init_pos, joint_pick_pos)
    # close gripper
    close_gripper()
    attach_flag = True
    rospy.sleep(2.0)
    thread.start_new_thread(attach_cube_thread, ("cube1", ))
    # place
    move_handle(joint_pick_pos, joint_middle_pos)
    move_handle(joint_middle_pos, joint_place_pos)
    # open gripper
    open_gripper()
    attach_flag = False
    rospy.sleep(3.0)
    # reset
    joint_pub_func(joint_arm_names, joint_init_pos)
    rospy.sleep(5.0)
