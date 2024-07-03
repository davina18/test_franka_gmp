#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import math
from PIL import Image
from tf import TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


## END_SUB_TUTORIAL

label_imgs = np.zeros((720, 1280), dtype=np.uint8)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)


def rotationMatrixToEulerAngles(R):
    if R[1, 0] > 0.998:
        x = 0
        y = math.pi / 2
        z = math.atan2(R[0, 2], R[2, 2])
    elif R[1, 0] < -0.998:
        x = 0
        y = -math.pi / 2
        z = math.atan2(R[0, 2], R[2, 2])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.asin(R[1, 0])
        z = math.atan2(-R[2, 0], R[0, 0])

    return x, y, z


def getRotationAndPosition(transformation):
    assert transformation.shape[0] == 4 and transformation.shape[1] == 4, "shape error"
    x = transformation[0, 3]
    y = transformation[1, 3]
    z = transformation[2, 3]
    x_r, y_r, z_r = rotationMatrixToEulerAngles(transformation[0:3, 0:3])
    print(x, y, z, x_r, y_r, z_r)
    return x, y, z, x_r, y_r, z_r


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


def pose_callback(cv_pose):
    # Convert the OpenCV pose to a PoseStamped message

    opencv_pose = PoseStamped()
    opencv_pose.header.frame_id = "camera_left_ir_optical_frame"
    opencv_pose.header.stamp = rospy.Time.now()

    Quaternion = get_quaternion_from_euler(
        np.double(cv_pose[3]), np.double(cv_pose[4]), np.double(cv_pose[5])
    )
    opencv_pose.pose.position.x = cv_pose[0]
    opencv_pose.pose.position.y = cv_pose[1]
    opencv_pose.pose.position.z = cv_pose[2]

    opencv_pose.pose.orientation.x = Quaternion[0]
    opencv_pose.pose.orientation.y = Quaternion[1]
    opencv_pose.pose.orientation.z = Quaternion[2]
    opencv_pose.pose.orientation.w = Quaternion[3]

    # opencv_pose.header.stamp = rospy.Time(0.60)
    # Transform the pose to the robot base coordinate system
    listener = TransformListener()
    listener.waitForTransform(
        "/camera_left_ir_optical_frame", "/world", rospy.Time(), rospy.Duration(2.0)
    )
    base_pose = listener.transformPose("world", opencv_pose)
    print("Translating the pose from camera to world")
    print(base_pose)
    # Process the converted pose information as required
    # process_pose(base_pose)
    return base_pose


def move_to_open_position():
    move_group_gripper = moveit_commander.MoveGroupCommander("panda_hand")
    open_joint_positions = [0.04, 0.04]  # Specify joint positions for the open gripper

    move_group_gripper.go(open_joint_positions, wait=True)
    move_group_gripper.stop()
    rospy.sleep(1.0)  # Wait for the movement to complete


def move_to_close_position():
    move_group_gripper = moveit_commander.MoveGroupCommander("panda_hand")
    closed_joint_positions = [
        0.0,
        0.0,
    ]  # Specify joint positions for the closed gripper

    move_group_gripper.go(closed_joint_positions, wait=True)
    move_group_gripper.stop()
    rospy.sleep(1.0)  # Wait for the movement to complete


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        group_gripper = "panda_hand"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_pose_reference_frame("world")
        gripper_pub = rospy.Publisher(
            "/panda/gripper/trajectory", JointTrajectory, queue_size=10
        )
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.gripper_pub = gripper_pub

    def open_gripper(self):
        print("opengripper")
        gripper_pub = self.gripper_pub
        gripper_command = JointTrajectory()
        gripper_command.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]

        open_joint_positions = [
            0.4,
            0.4,
        ]  # Specify joint positions to open the gripper

        gripper_command.points.append(
            JointTrajectoryPoint(
                positions=open_joint_positions, time_from_start=rospy.Duration(1.0)
            )
        )
        gripper_pub.publish(gripper_command)
        rospy.sleep(2.5)  # Wait for the gripper to finish opening

    def close_gripper(self):
        gripper_pub = self.gripper_pub
        gripper_command = JointTrajectory()
        gripper_command.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]

        closed_joint_positions = [
            0.0,
            0.0,
        ]  # Specify joint positions to close the gripper

        gripper_command.points.append(
            JointTrajectoryPoint(
                positions=closed_joint_positions, time_from_start=rospy.Duration(1.0)
            )
        )
        gripper_pub.publish(gripper_command)
        rospy.sleep(2.5)  # Wait for the gripper to finish closing

    def go_to_pose_goal(self, pose):
        move_group = self.move_group
        self.pose = pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal = pose
        move_group.set_pose_target(pose_goal)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose

        return all_close(pose_goal, current_pose, 0.01)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)


def main():
    b = np.load(
        "/home/furkan/contact_graspnet/results/predictions_franka_gazebo.npz",
        allow_pickle=True,
        encoding="bytes",
    )
    points = b["pred_grasps_cam"]
    grasp_points = points.item()[-1]
    pose = getRotationAndPosition(grasp_points[60])
    base_poses = pose_callback(pose)
    place_pose = [0]
    tutorial = MoveGroupPythonInterfaceTutorial()
    move_to_open_position()
    input("============ Press `Enter` to execute a movement using a pose goal ...")
    tutorial.go_to_pose_goal(base_poses)
    move_to_close_position()
    # tutorial.go_to_pose_goal(place_pose)


if __name__ == "__main__":
    main()
