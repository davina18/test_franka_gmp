#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Bool
import pickle

class move_to_capture:

    def __init__(self):
        return


if __name__ == "__main__":
    rospy.init_node("move_to_capture_node")

    moveit_commander.roscpp_initialize(sys.argv)

    # ! INITIALIZE PUBLISHER
    pub = rospy.Publisher("capture_reached", Bool, queue_size=10)

    # ! INITIALIZE MOVEIT CLASSES
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.set_end_effector_link(group.get_end_effector_link())
    group.set_planning_time(10)
    group.set_goal_tolerance(0.01)
    goal_reached = True
    
    # ! ADD COLLISION OBJECTS
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.position.x = 0.00
    box_pose.pose.position.y = 0.00
    box_pose.pose.position.z = 0.71
    box_pose.pose.orientation.x = 0.00
    box_pose.pose.orientation.y = 0.00
    box_pose.pose.orientation.z = 0.00
    box_pose.pose.orientation.w = 1.00
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.8, 1.58, 0.019708))
    
    # ! DEFINE THE GOAL
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.1786826546916537
    pose_goal.position.y = 0.13570680784299066
    pose_goal.position.z = 1.0128471408708453

    pose_goal.orientation.x = -0.6540759979798239
    pose_goal.orientation.y = -0.28296212139976934
    pose_goal.orientation.z = -0.26436570905183826
    pose_goal.orientation.w = 0.6497905805697389

    group.set_pose_target(pose_goal)

    # ! SEND TO THE GOAL
    saved_path = "pickled_plan.yaml"

    with open(saved_path, 'rb') as file_open:
        plan = pickle.load(file_open)[1]

    group.execute(plan, wait=True)

    rospy.loginfo("Capture Position Reached!")
    
    group.stop()
    group.clear_pose_targets()

    # ! PUBLISH COMPLETION
    pub.publish(True)

