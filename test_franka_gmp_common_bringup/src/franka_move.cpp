// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Gazebo Attacher
#ifdef GAZEBO_LINK_ATTACHER
#include <gazebo_ros_link_attacher/Attach.h>
#endif


// Services
#ifdef GAZEBO_LINK_ATTACHER
  gazebo_ros_link_attacher::Attach attach_srv_;
  ros::ServiceClient attach_client_, detach_client_;
#endif


// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// Some definitions
static const std::string PLANNING_GROUP_ARM = "panda_arm";
static const std::string PLANNING_GROUP_GRIPPER = "panda_hand";

//to open Gripper
void openGripper()
{
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
  move_group_interface_gripper.move();
  ROS_INFO("OpenGripper");

}

//to close Gripper
void closedGripper()
{
  

  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close"));
  move_group_interface_gripper.move();
  ROS_INFO("ClosedGripper");
  
}


// to move a specific point with specific angles
void move_pos(double rpy[3] = { }, double positions[3] = { } )
{

  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  geometry_msgs::Pose target_pose1;
  tf2::Quaternion orientation;
  orientation.setRPY(rpy[0], rpy[1], rpy[2]);
  target_pose1.orientation = tf2::toMsg(orientation);
  target_pose1.position.x = positions[0];
  target_pose1.position.y = positions[1];
  target_pose1.position.z = positions[2];
  move_group_interface_arm.setPoseTarget(target_pose1);
  move_group_interface_arm.move();
  ROS_INFO("!!!Moving!!!");

}

// to attach the object to the gripper
void attach1(moveit::planning_interface::MoveGroupInterface& move_group_atc)
{

  // Now, let's attach the collision object to the robot.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group_atc.attachObject("coke_can", "panda_leftfinger",
                                  {"panda_rightfinger", "panda_leftfinger",
                                   "panda_hand"
                                  "link"});
#ifdef GAZEBO_LINK_ATTACHER
  attach_srv_.request.model_name_1 = "coke_can";
  attach_srv_.request.link_name_1 = "link";

  if (attach_client_.call(attach_srv_))
    ROS_INFO("Successfully attached");
  else
    ROS_WARN("NOT attached");
#endif
}

// to detach the object from the gripper
void detach1(moveit::planning_interface::MoveGroupInterface& move_group_atc)
{

  ROS_INFO_NAMED("tutorial", "Detach the object to the robot");
  move_group_atc.detachObject("coke_can");
  
#ifdef GAZEBO_LINK_ATTACHER
  if (detach_client_.call(attach_srv_))
      ROS_INFO("Successfully detached");
  else
      ROS_WARN("NOT detached");
#endif
}



void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment in Rviz
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 2 collision objects(table and object).
  
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table";

  collision_objects[0].header.frame_id = "world";

  /* Define the primitive and its dimensions. */

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.5;
  collision_objects[0].primitives[0].dimensions[1] = 1.6;
  collision_objects[0].primitives[0].dimensions[2] = 1;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.4;
  collision_objects[0].primitive_poses[0].position.y = 0.02;
  collision_objects[0].primitive_poses[0].position.z = 0.5;
  collision_objects[0].primitive_poses[0].orientation.z = 0;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Define the object that we will be manipulating
  collision_objects[1].id = "coke_can";
  collision_objects[1].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
  collision_objects[1].primitives[0].dimensions.resize(2);
  collision_objects[1].primitives[0].dimensions[0] = 0.13;
  collision_objects[1].primitives[0].dimensions[1] = 0.034;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.4;
  collision_objects[1].primitive_poses[0].position.y = 0.4;
  collision_objects[1].primitive_poses[0].position.z = 1.04;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

//=========================================================
// Services
//=========================================================
#ifdef GAZEBO_LINK_ATTACHER
    attach_srv_.request.model_name_2 = "panda";
    attach_srv_.request.link_name_2 = "panda_leftfinger";


    attach_client_ = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_"
                                                                       "attacher_"
                                                                       "node/"
                                                                       "attach");

    detach_client_ = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_"
                                                                       "attacher_"
                                                                       "node/"
                                                                       "detach");
#endif


  addCollisionObjects(planning_scene_interface);
    
  // Moving to pre-grasp position
  openGripper();
  double pre_grasp_rpy[3] = {-tau/4, -tau/8, -tau/8};
  double pre_grasp_pose[3] = {0.25, 0.25, 1.125};
  move_pos(pre_grasp_rpy, pre_grasp_pose);

  // Moving to grasp position and grasp the object
  double grasp_pose[3] = {0.33, 0.33, 1.125};
  move_pos(pre_grasp_rpy, grasp_pose);
  closedGripper();
  attach1(move_group_interface_arm);

  // Moving to post-grasp position
  double post_grasp_pose[3] = {0.33, 0.33, 1.25};
  move_pos(pre_grasp_rpy, post_grasp_pose);

  //Moving to pre-place position
  double place_rpy[3] = {-tau / 4, -tau / 8, -tau / 4};
  double pre_place_pose[3] = {0.33, -0.5, 1.2};
  move_pos(place_rpy, pre_place_pose);

  //Moving to place position and place the object
  double place_pose[3] = {0.33, -0.5, 1.15};
  move_pos(place_rpy, place_pose);
  openGripper();
  detach1(move_group_interface_arm);

  //Moving to home position
  double home_rpy[3] = {-tau/4, -tau/8, -tau/4};
  double home_pose[3] = {0.2, 0.02, 1.5};
  move_pos(home_rpy, home_pose);

  ros::waitForShutdown();
  return 0;
}
