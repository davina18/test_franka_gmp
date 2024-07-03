#include <iostream>
#include <vector>
#include <sstream>
#include <algorithm>
#include <regex>
#include <fstream>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include <memory>

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CollisionObject.h>

// TF
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <visualization_msgs/Marker.h>

using namespace std;

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// Some definitions
static const std::string PLANNING_GROUP_ARM = "panda_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "panda_hand";

// Define the structure ObjectGrasp
struct ObjectGrasp
{
    double position[3];
    double orientation[4];
    double scores;
};

std::vector<ObjectGrasp> objectList;

// Custom predicate function to check if a character is whitespace
bool is_whitespace(char ccc)
{
    return std::isspace(static_cast<unsigned char>(ccc));
}

bool compareByScore(const ObjectGrasp &a, const ObjectGrasp &b)
{
    return a.scores > b.scores;
}

void printMatrix(double matrix[4][4])
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void convertStringtoDouble(std::string pred_grasp_string)
{
    ObjectGrasp object;
    Eigen::Matrix3d rotationMatrix;
    // Remove all non-numeric characters except '-' and '.' using regular expression
    pred_grasp_string = std::regex_replace(pred_grasp_string, std::regex("[^0-9\\.\\,\\- ]"), "");

    // Split the string into rows using ',' as the delimiter
    std::vector<double> rows;
    std::stringstream ss(pred_grasp_string);
    std::string row;
    while (std::getline(ss, row, ','))
    {
        double value = stod(row);
        rows.push_back(value);
    }
    if (rows.size() != 20)
    {
        std::cerr << "Error: Invalid number of elements in the string." << std::endl;
    }

    // Convert the vector to a 2D array (4x4 matrix in this case)
    double transformationMatrix[4][4];
    int index = 0;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            transformationMatrix[i][j] = rows[index++];
        }
    }
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rotationMatrix(i, j) = transformationMatrix[i][j];
        }
    }
    // std::cout << rotationMatrix << std::endl;
    object.position[0] = transformationMatrix[0][3];
    object.position[1] = transformationMatrix[1][3];
    object.position[2] = transformationMatrix[2][3];
    // object.position[0] = rows[17];
    // object.position[1] = rows[18];
    // object.position[2] = rows[19];
    object.scores = rows[16];
    Eigen::Quaterniond quaternion(rotationMatrix);
    object.orientation[0] = quaternion.x();
    object.orientation[1] = quaternion.y();
    object.orientation[2] = quaternion.z();
    object.orientation[3] = quaternion.w();
    objectList.push_back(object);
}

void getGraspPoints()
{
    std::string username = std::getenv("USER");
    std::string fname = "/home/furkan/ros/noetic/repos/github.com/CardiffUniversityComputationalRobotics/hybridplanner-goal-regions/hybridplanner_common_bringup/src/grasping_points.csv";

    // std::string fname = "../../hybridplanner-goal-regions/hybridplanner_common_bringup/src/results/grasping_points.csv";

    vector<vector<string>>
        content,
        content_s;
    vector<string> row;
    string line, word;

    fstream file(fname, ios::in);
    if (file.is_open())
    {
        while (getline(file, line))
        {
            row.clear();

            stringstream str(line);

            while (getline(str, word))
                row.push_back(word);
            content.push_back(row);
        }
    }
    else
        cout << "Could not open the file\n";

    for (size_t i = 1; i < content.size(); i++)
    {
        // cout << content[i][0] << endl;
        convertStringtoDouble(content[i][0]);
    }
}
// void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
//                          ros::Publisher &marker_pub)
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Define TableAgr
    moveit_msgs::CollisionObject table_object;
    table_object.header.frame_id = "world";
    table_object.id = "TableAgr";
    table_object.primitives.resize(1);
    table_object.primitives[0].type = table_object.primitives[0].BOX;
    table_object.primitives[0].dimensions.resize(3);
    table_object.primitives[0].dimensions[0] = 1.406120;
    table_object.primitives[0].dimensions[1] = 0.8;
    table_object.primitives[0].dimensions[2] = 0.719708;
    table_object.primitive_poses.resize(1);
    table_object.primitive_poses[0].position.x = -0.022089;
    table_object.primitive_poses[0].position.y = -0.25;
    table_object.primitive_poses[0].position.z = 0.359854;
    table_object.primitive_poses[0].orientation.x = 0;
    table_object.primitive_poses[0].orientation.y = 0;
    table_object.primitive_poses[0].orientation.z = 0.706825181105366;
    table_object.primitive_poses[0].orientation.w = 0.7073882691671998;
    table_object.operation = table_object.ADD;
    collision_objects.push_back(table_object);

    // Define BoxAgr1
    moveit_msgs::CollisionObject box1_object;
    box1_object.header.frame_id = "world";
    box1_object.id = "BoxAgr1";
    box1_object.primitives.resize(1);
    box1_object.primitives[0].type = box1_object.primitives[0].BOX;
    box1_object.primitives[0].dimensions.resize(3);
    box1_object.primitives[0].dimensions[0] = 0.04;
    box1_object.primitives[0].dimensions[1] = 0.04;
    box1_object.primitives[0].dimensions[2] = 0.2;
    box1_object.primitive_poses.resize(1);
    box1_object.primitive_poses[0].position.x = -0.2699960413639732;
    box1_object.primitive_poses[0].position.y = 0.6799937560697008;
    box1_object.primitive_poses[0].position.z = 1.0479988105046956;
    box1_object.primitive_poses[0].orientation.x = 1.1832213590485467e-05;
    box1_object.primitive_poses[0].orientation.y = 6.734332507288247e-05;
    box1_object.primitive_poses[0].orientation.z = 0.0007308084147995393;
    box1_object.primitive_poses[0].orientation.w = 0.9999997306219318;
    box1_object.operation = box1_object.ADD;
    collision_objects.push_back(box1_object);

    // Define BoxAgrObs1
    moveit_msgs::CollisionObject box_obs1_object;
    box_obs1_object.header.frame_id = "world";
    box_obs1_object.id = "BoxAgrObs1";
    box_obs1_object.primitives.resize(1);
    box_obs1_object.primitives[0].type = box_obs1_object.primitives[0].BOX;
    box_obs1_object.primitives[0].dimensions.resize(3);
    box_obs1_object.primitives[0].dimensions[0] = 0.15;
    box_obs1_object.primitives[0].dimensions[1] = 0.2;
    box_obs1_object.primitives[0].dimensions[2] = 0.15;
    box_obs1_object.primitive_poses.resize(1);
    box_obs1_object.primitive_poses[0].position.x = 0.06990756034828235;
    box_obs1_object.primitive_poses[0].position.y = 0.7999753237036121;
    box_obs1_object.primitive_poses[0].position.z = 1.0229899132423599;
    box_obs1_object.primitive_poses[0].orientation.x = 1.718305483178145e-05;
    box_obs1_object.primitive_poses[0].orientation.y = 3.537885020202517e-05;
    box_obs1_object.primitive_poses[0].orientation.z = 8.417541989761031e-05;
    box_obs1_object.primitive_poses[0].orientation.w = 0.9999999956837892;
    box_obs1_object.operation = box_obs1_object.ADD;
    collision_objects.push_back(box_obs1_object);

    // Define CylinderAgr2
    moveit_msgs::CollisionObject cylinder2_object;
    cylinder2_object.header.frame_id = "world";
    cylinder2_object.id = "CylinderAgr2";
    cylinder2_object.primitives.resize(1);
    cylinder2_object.primitives[0].type = cylinder2_object.primitives[0].CYLINDER;
    cylinder2_object.primitives[0].dimensions.resize(2);
    cylinder2_object.primitives[0].dimensions[0] = 0.301258;
    cylinder2_object.primitives[0].dimensions[1] = 0.017770;
    cylinder2_object.primitive_poses.resize(1);
    cylinder2_object.primitive_poses[0].position.x = 0.26741896393277953;
    cylinder2_object.primitive_poses[0].position.y = 0.8029563120648224;
    cylinder2_object.primitive_poses[0].position.z = 1.0427007996282558;
    cylinder2_object.primitive_poses[0].orientation.x = -0.014554564814981933;
    cylinder2_object.primitive_poses[0].orientation.y = -0.4845382468762161;
    cylinder2_object.primitive_poses[0].orientation.z = 0.026256394834823175;
    cylinder2_object.primitive_poses[0].orientation.w = 0.8742547990645791;
    cylinder2_object.operation = cylinder2_object.ADD;
    collision_objects.push_back(cylinder2_object);

    // Define CylinderAgr1
    moveit_msgs::CollisionObject cylinder1_object;
    cylinder1_object.header.frame_id = "world";
    cylinder1_object.id = "CylinderAgr1";
    cylinder1_object.primitives.resize(1);
    cylinder1_object.primitives[0].type = cylinder1_object.primitives[0].CYLINDER;
    cylinder1_object.primitives[0].dimensions.resize(2);
    cylinder1_object.primitives[0].dimensions[0] = 0.21;
    cylinder1_object.primitives[0].dimensions[1] = 0.021;
    cylinder1_object.primitive_poses.resize(1);
    cylinder1_object.primitive_poses[0].position.x = 0.07081782825251383;
    cylinder1_object.primitive_poses[0].position.y = 0.522705294984615;
    cylinder1_object.primitive_poses[0].position.z = 1.0479974962212406;
    cylinder1_object.primitive_poses[0].orientation.x = 2.5449800432384e-05;
    cylinder1_object.primitive_poses[0].orientation.y = -1.7224632961150036e-05;
    cylinder1_object.primitive_poses[0].orientation.z = -8.32485851179764e-06;
    cylinder1_object.primitive_poses[0].orientation.w = 0.9999999994931582;
    cylinder1_object.operation = cylinder1_object.ADD;
    collision_objects.push_back(cylinder1_object);

    // // Define CylinderAgrObs1
    // moveit_msgs::CollisionObject cylinder_obs1_object;
    // cylinder_obs1_object.header.frame_id = "world";
    // cylinder_obs1_object.id = "CylinderAgrObs1";
    // cylinder_obs1_object.primitives.resize(1);
    // cylinder_obs1_object.primitives[0].type = cylinder_obs1_object.primitives[0].CYLINDER;
    // cylinder_obs1_object.primitives[0].dimensions.resize(2);
    // cylinder_obs1_object.primitives[0].dimensions[0] = 0.098434;
    // cylinder_obs1_object.primitives[0].dimensions[1] = 0.042077;
    // cylinder_obs1_object.primitive_poses.resize(1);
    // cylinder_obs1_object.primitive_poses[0].position.x = -0.09992073366968887;
    // cylinder_obs1_object.primitive_poses[0].position.y = 0.5502146720147691;
    // cylinder_obs1_object.primitive_poses[0].position.z = 0.9972088267726038;
    // cylinder_obs1_object.primitive_poses[0].orientation.x = 1.4653061498520588e-05;
    // cylinder_obs1_object.primitive_poses[0].orientation.y = -9.853536147419642e-06;
    // cylinder_obs1_object.primitive_poses[0].orientation.z = -0.00044104549862902776;
    // cylinder_obs1_object.primitive_poses[0].orientation.w = 0.9999999025835271;
    // cylinder_obs1_object.operation = cylinder_obs1_object.ADD;
    // collision_objects.push_back(cylinder_obs1_object);

    // // Define CylinderAgrObs2
    // moveit_msgs::CollisionObject cylinder_obs2_object;
    // cylinder_obs2_object.header.frame_id = "world";
    // cylinder_obs2_object.id = "CylinderAgrObs2";
    // cylinder_obs2_object.primitives.resize(1);
    // cylinder_obs2_object.primitives[0].type = cylinder_obs2_object.primitives[0].CYLINDER;
    // cylinder_obs2_object.primitives[0].dimensions.resize(2);
    // cylinder_obs2_object.primitives[0].dimensions[0] = 0.098434;
    // cylinder_obs2_object.primitives[0].dimensions[1] = 0.042077;
    // cylinder_obs2_object.primitive_poses.resize(1);
    // cylinder_obs2_object.primitive_poses[0].position.x = -0.2863780969352303;
    // cylinder_obs2_object.primitive_poses[0].position.y = 0.5578850082189986;
    // cylinder_obs2_object.primitive_poses[0].position.z = 0.9972040903304636;
    // cylinder_obs2_object.primitive_poses[0].orientation.x = 0;
    // cylinder_obs2_object.primitive_poses[0].orientation.y = 0;
    // cylinder_obs2_object.primitive_poses[0].orientation.z = 0;
    // cylinder_obs2_object.primitive_poses[0].orientation.w = 1;
    // cylinder_obs2_object.operation = cylinder_obs2_object.ADD;
    // collision_objects.push_back(cylinder_obs2_object);

    // // Define CylinderAgrObs3
    // moveit_msgs::CollisionObject cylinder_obs3_object;
    // cylinder_obs3_object.header.frame_id = "world";
    // cylinder_obs3_object.id = "CylinderAgrObs3";
    // cylinder_obs3_object.primitives.resize(1);
    // cylinder_obs3_object.primitives[0].type = cylinder_obs3_object.primitives[0].CYLINDER;
    // cylinder_obs3_object.primitives[0].dimensions.resize(2);
    // cylinder_obs3_object.primitives[0].dimensions[0] = 0.098434;
    // cylinder_obs3_object.primitives[0].dimensions[1] = 0.042077;
    // cylinder_obs3_object.primitive_poses.resize(1);
    // cylinder_obs3_object.primitive_poses[0].position.x = 0.2784008134337255;
    // cylinder_obs3_object.primitive_poses[0].position.y = 0.5496231879454214;
    // cylinder_obs3_object.primitive_poses[0].position.z = 0.9972025688468263;
    // cylinder_obs3_object.primitive_poses[0].orientation.x = -1.7319394332204237e-05;
    // cylinder_obs3_object.primitive_poses[0].orientation.y = 6.346635490838681e-07;
    // cylinder_obs3_object.primitive_poses[0].orientation.z = -0.0004091532405142554;
    // cylinder_obs3_object.primitive_poses[0].orientation.w = 0.9999999161466273;
    // cylinder_obs3_object.operation = cylinder_obs3_object.ADD;
    // collision_objects.push_back(cylinder_obs3_object);

    // Define book_shelf
    moveit_msgs::CollisionObject shelf_object;
    string username = std::getenv("USER");
    shapes::Mesh *m = shapes::createMeshFromResource(
        "file:///home/" + username + "/gazebo_models/" + "shelf/shelf_v5.stl");
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::Mesh mesh;
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    shelf_object.header.frame_id = "world";
    shelf_object.mesh_poses.resize(1);

    shelf_object.mesh_poses[0].position.x = -0.01999999576323767;
    shelf_object.mesh_poses[0].position.y = 0.4700970008977307;
    shelf_object.mesh_poses[0].position.z = -1.5857498663710862e-06;
    shelf_object.mesh_poses[0].orientation.x = -2.5620643484206315e-08;
    shelf_object.mesh_poses[0].orientation.y = 1.42925349112172e-07;
    shelf_object.mesh_poses[0].orientation.z = 4.495408494746955e-06;
    shelf_object.mesh_poses[0].orientation.w = 0.9999999999898852;
    shelf_object.meshes.push_back(mesh);
    shelf_object.operation = shelf_object.ADD;
    collision_objects.push_back(shelf_object);
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

// Function to apply rotation
geometry_msgs::Pose applyCoordinateTransform(const std::vector<double> &cv_pose)
{
    // Define the rotation (90 degrees around Z)
    double angle_degrees = 90;                             // angle in degrees
    double angle_radians = M_PI * (angle_degrees / 180.0); // converting degrees to radians

    // Creating a rotation matrix for 90 degrees rotation around Z-axis
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(angle_radians, Eigen::Vector3d::UnitZ());

    // If you need to convert this rotation matrix to a quaternion for further use:
    Eigen::Quaterniond rotation(rotation_matrix);
    // Convert OpenCV quaternion to Eigen quaternion
    Eigen::Quaterniond opencv_quaternion(cv_pose[6],
                                         cv_pose[3],
                                         cv_pose[4],
                                         cv_pose[5]);
    // Apply the rotation
    Eigen::Quaterniond transformed_quaternion = opencv_quaternion * rotation;
    transformed_quaternion.normalize(); // Normalize the quaternion
    // Convert back to ROS pose
    geometry_msgs::Pose transformed_pose;
    transformed_pose.orientation.x = transformed_quaternion.x();
    transformed_pose.orientation.y = transformed_quaternion.y();
    transformed_pose.orientation.z = transformed_quaternion.z();
    transformed_pose.orientation.w = transformed_quaternion.w();

    return transformed_pose;
}

geometry_msgs::PoseStamped pose_callback(const std::vector<double> &cv_pose)
{
    // Convert the OpenCV pose to a PoseStamped message
    geometry_msgs::PoseStamped opencv_pose;
    opencv_pose.pose = applyCoordinateTransform(cv_pose);
    opencv_pose.header.frame_id = "camera_depth_optical_frame";
    opencv_pose.header.stamp = ros::Time::now();

    opencv_pose.pose.position.x = cv_pose[0];
    opencv_pose.pose.position.y = cv_pose[1];
    opencv_pose.pose.position.z = cv_pose[2];

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);

    try
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("world", "camera_depth_optical_frame", ros::Time(0), ros::Duration(1.0));

        geometry_msgs::PoseStamped base_pose;
        tf2::doTransform(opencv_pose, base_pose, transformStamped);

        ROS_INFO("Translating the pose from camera to world");
        ROS_INFO_STREAM(base_pose);

        return base_pose;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        // Handle the exception if the transform is not available or other issues
        // Return the original opencv_pose or an appropriate default pose
        return opencv_pose;
    }
}

// to open Gripper
void openGripper()
{
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
    move_group_interface_gripper.move();
    ROS_INFO("OpenGripper");
}

// to close Gripper
void closedGripper()
{

    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close"));
    move_group_interface_gripper.move();
    ROS_INFO("ClosedGripper");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_arm_pick_place_goalregions");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 101);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveItErrorCode success =
        moveit::planning_interface::MoveItErrorCode::FAILURE;
    auto pcm = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(pcm);
    // Moving to pre-grasp position
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Rate loop_rate(10);
    openGripper();
    getGraspPoints();
    bool isRunning = true;
    // for (const auto &object : objectList[2])
    // {
    std::sort(objectList.begin(), objectList.end(), compareByScore);
    std::vector<geometry_msgs::PoseStamped> targetPoses;

    string obj;
    string referance_frame = "world";
    addCollisionObjects(planning_scene_interface);

    // addCollisionObjects(planning_scene_interface, marker_pub);
    move_group.setJointValueTarget({1.57, -1.57, 0.0, -1.57, 0.0, 1.13446401, 0.7854});
    move_group.move();
    // cout << "press any button" << endl;
    // getline(cin, obj);
    move_group.clearPoseTargets();
    const auto &object = objectList[0];

    int number = 100; // Set 'number' to however many top poses you want
    for (int i = 0; i < number && i < objectList.size(); ++i)
    {
        const auto &object = objectList[i];
        std::cout << "Score of pose " << (i + 1) << ": " << object.scores << std::endl;
        std::vector<double> pose = {object.position[0], object.position[1], object.position[2],
                                    object.orientation[0], object.orientation[1],
                                    object.orientation[2], object.orientation[3]};
        geometry_msgs::PoseStamped base_poses = pose_callback(pose);
        base_poses.header.frame_id = "world";
        targetPoses.push_back(base_poses);
    }
    move_group.setPlannerId("RRTConnect");
    move_group.setPlanningTime(20.0);
    int NFail, NAttempts;
    NFail = 0;
    NAttempts = 0;
    move_group.setPoseTargets(targetPoses, "panda_hand_tcp");
    do
    {
        NAttempts = NAttempts + 1;
        success = move_group.plan(my_plan);

        if (success)
        {
            ROS_INFO("Succeeded planning pre grasp");
            success = move_group.execute(my_plan);
            sleep(1.0);
            std::cout << "Moving back to home" << std::endl;
            move_group.setJointValueTarget({1.57, -1.57, 0.0, -1.57, 0.0, 1.13446401, 0.7854});
            move_group.move();
            move_group.setPoseTargets(targetPoses, "panda_hand_tcp");
            cout << "Fail/Attempts: " << NFail << " / " << NAttempts << endl;

            if (!success)
            {
                ROS_WARN("Failed executing pre grasp");
            }
        }
        else
        {
            ROS_WARN("Failed planning pre grasp");
            NFail = NFail + 1;
            cout << "Fail/Attempts: " << NFail << " / " << NAttempts << endl;
        }

        loop_rate.sleep();
    } while (ros::ok() && (NAttempts < 100));
}
