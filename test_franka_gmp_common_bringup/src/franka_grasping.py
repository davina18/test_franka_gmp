#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import subprocess

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


## END_SUB_TUTORIAL

label_imgs = np.zeros((720, 1280), dtype=np.uint8)


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
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
    """
    Convert an Euler angle to a quaternion.

    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
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


# Instantiate CvBridge
bridge = CvBridge()


def color_callback(msg):
    # print("Received color image!")
    global color_img
    color_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    print("saving color image")


def depth_callback(depth_msg):
    global depthimg
    bridge = CvBridge()
    depth_img = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    cv_image_array = np.array(depth_img, dtype=np.dtype("f8"))
    cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    # cv_image_resized = cv2.resize(cv_image_norm, interpolation=cv2.INTER_CUBIC)
    depthimg = depth_img
    print("saving depth image")


def camera_info_callback(msg):
    global K
    global aligned_depth_image
    cam_info = msg.K
    K = np.array(
        [
            [cam_info[0], 0.0, cam_info[2]],
            [0.0, cam_info[4], cam_info[5]],
            [0.0, 0.0, 0.0],
        ]
    )
    print("saving camera_parameters")


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
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_planner_id("RRTstar")

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
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
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

    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:

        pose_goal = geometry_msgs.msg.Pose()
        Quaternion = get_quaternion_from_euler(-2.48, -0.1, -0.6775759)
        pose_goal.orientation.x = Quaternion[0]
        pose_goal.orientation.y = Quaternion[1]
        pose_goal.orientation.z = Quaternion[2]
        pose_goal.orientation.w = Quaternion[3]
        # pose_goal.orientation.x = 0.64263
        # pose_goal.orientation.y = 0.60385
        # pose_goal.orientation.z = 0.34995
        # pose_goal.orientation.w = -0.31613
        pose_goal.position.x = 0.032498
        pose_goal.position.y = 0.22183
        pose_goal.position.z = 1.3121
        # move_group.set_pose_target(pose_goal)
        move_group.go([1.57, -1.57, 0.0, -1.57, 0.0, 1.13446401, 0.7854], wait=True)
        # move_group.go(wait=True)
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        # success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIARRT


def main():
    tutorial = MoveGroupPythonInterfaceTutorial()
    # input("============ Press `Enter` to execute a movement using a pose goal ...")
    tutorial.go_to_pose_goal()

    color_topic = "/camera/color/image_raw"
    rospy.Subscriber(color_topic, Image, color_callback)
    rospy.sleep(0.1)
    depth_topic = "/camera/aligned_depth_to_color/image_raw"
    rospy.Subscriber(depth_topic, Image, depth_callback)
    rospy.sleep(0.1)
    cam_topic = "/camera/aligned_depth_to_color/camera_info"
    rospy.Subscriber(cam_topic, CameraInfo, camera_info_callback)
    rospy.sleep(1)

    # Define the HSV range for the red color
    lower_red = np.array([0, 70, 50])
    upper_red = np.array([10, 255, 255])

    # Define the HSV range for the brown color (commonly associated with a 'shelf')
    lower_brown = np.array([10, 20, 20])  # Adjust these values if needed for your brown
    upper_brown = np.array([52, 255, 255])

    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)

    # Create a mask for the red regions
    mask_red = cv2.inRange(hsv_image, lower_red, upper_red)

    # Create a mask for the brown regions
    mask_brown = cv2.inRange(hsv_image, lower_brown, upper_brown)

    # Combine the masks for red and brown using OR - this will give us a mask where either red or brown is present
    mask_combined = cv2.bitwise_or(mask_red, mask_brown)

    # Invert the combined mask to get the non-red and non-brown areas
    mask_non_red_brown = cv2.bitwise_not(mask_combined)

    # Apply the non-red and non-brown mask to the HSV image to get objects that are neither red nor brown
    non_red_brown_objects_hsv = cv2.bitwise_and(
        hsv_image, hsv_image, mask=mask_non_red_brown
    )

    # Convert back to RGB to display the objects that are not red or brown
    non_red_brown_objects_rgb = cv2.cvtColor(
        non_red_brown_objects_hsv, cv2.COLOR_HSV2BGR
    )

    # Display the objects that are not red or brown
    cv2.imshow("Non-Red and Non-Brown Objects", color_img)
    cv2.waitKey(0)

    # Apply the non-red and non-brown mask to the depth image
    filtered_depth_image = cv2.bitwise_and(depthimg, depthimg, mask=mask_non_red_brown)

    # Display the filtered depth image
    cv2.imshow("Filtered Depth Image", filtered_depth_image)
    cv2.waitKey(0)
    # align_depth_to_color()
    data_dict = {
        "rgb": np.array(color_img),
        "depth": np.array(filtered_depth_image) / 1000.0,
        "label": label_imgs,
        "K": K
        #    "seg": segmentation_map,
    }
    np.save("/home/furkan/contact_graspnet/test_data/franka_gazebo.npy", data_dict)
    print("saved")
    # subprocess.run(["bash", script_path])


if __name__ == "__main__":
    main()
