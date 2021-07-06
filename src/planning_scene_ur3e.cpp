// Author: Prasanth Suresh(ps32611@uga.edu); 
// Description: Adding surrounding collision objects to ur3e world in Moveit; 
// Do not edit/copy without permission.


#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Int32.h"

using namespace Eigen;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold num_objects number of collision objects.
  int num_objects = 1;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(num_objects);
  collision_objects[0].id = "conveyor_table";
  collision_objects[0].header.frame_id = "world";
  shapes::Mesh* m = shapes::createMeshFromResource("package://ur3e_irl_project/meshes/conveyor_box.stl"); 
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;  
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
collision_objects[0].mesh_poses[0].position.x = 1.0;
  collision_objects[0].mesh_poses[0].position.y = 1.0;
  collision_objects[0].mesh_poses[0].position.z = 1.0;
  collision_objects[0].mesh_poses[0].orientation.w = 1.0; 
  collision_objects[0].mesh_poses[0].orientation.x = 0.0; 
  collision_objects[0].mesh_poses[0].orientation.y = 0.0;
  collision_objects[0].mesh_poses[0].orientation.z = 0.0;   
  collision_objects[0].meshes.push_back(mesh);
  collision_objects[0].mesh_poses.push_back(collision_objects[0].mesh_poses[0]);
  collision_objects[0].operation = collision_objects[0].ADD;
  ROS_INFO("Conveyor box mesh loaded");

  collision_objects[0].id = "conveyor_table";
  collision_objects[0].header.frame_id = "world";
  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.45;
  primitive.dimensions[1] = 1.5;
  primitive.dimensions[2] = 0.75;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.75;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.46;

  collision_objects[0].primitives.push_back(primitive);
  collision_objects[0].primitive_poses.push_back(box_pose);
  collision_objects[0].operation = collision_objects[0].ADD;
  ROS_INFO("Conveyor box loaded");
  
  // // Add the railing2 above the conveyor.
  // collision_objects[1].id = "railing1";
  // collision_objects[1].header.frame_id = "world";
  // /* Define a box to add to the world. */
  // shape_msgs::SolidPrimitive primitive1;
  // primitive1.type = primitive1.BOX;
  // primitive1.dimensions.resize(3);
  // primitive1.dimensions[0] = 0.01;
  // primitive1.dimensions[1] = 1.5;
  // primitive1.dimensions[2] = 0.01;

  // /* A pose for the box (specified relative to frame_id) */
  // geometry_msgs::Pose box_pose1;
  // box_pose1.orientation.w = 1.0;
  // box_pose1.position.x =  0.95;
  // box_pose1.position.y = 0.0;
  // box_pose1.position.z = -0.08;

  // collision_objects[1].primitives.push_back(primitive1);
  // collision_objects[1].primitive_poses.push_back(box_pose1);
  // collision_objects[1].operation = collision_objects[1].ADD;
  // ROS_INFO("Railing 1 loaded");

  // // Add the onion_bin where the sorted onions will be kept.
  // collision_objects[2].id = "onion_bin";
  // collision_objects[2].header.frame_id = "world";
  // shapes::Mesh* m2 = shapes::createMeshFromResource("package://ur3e_irl_project/meshes/onion_bin.stl"); 
  // ROS_INFO("Onion bin mesh loaded");
  // shape_msgs::Mesh mesh2;
  // shapes::ShapeMsg mesh_msg2;  
  // shapes::constructMsgFromShape(m2, mesh_msg2);
  // mesh2 = boost::get<shape_msgs::Mesh>(mesh_msg2);
  // collision_objects[2].meshes.resize(1); 
  // collision_objects[2].mesh_poses.resize(1);  
  // collision_objects[2].mesh_poses[0].position.x = 0.1;
  // collision_objects[2].mesh_poses[0].position.y = 0.6;
  // collision_objects[2].mesh_poses[0].position.z = -0.9;
  // collision_objects[2].mesh_poses[0].orientation.w= 1.0; 
  // collision_objects[2].mesh_poses[0].orientation.x= 0.0; 
  // collision_objects[2].mesh_poses[0].orientation.y= 0.0;
  // collision_objects[2].mesh_poses[0].orientation.z= 0.0;   
  // collision_objects[2].meshes.push_back(mesh2);
  // collision_objects[2].mesh_poses.push_back(collision_objects[2].mesh_poses[0]);
  // collision_objects[2].operation = collision_objects[2].ADD;

  // // Add the railing1 above the conveyor.
  // collision_objects[3].id = "railing2";
  // collision_objects[3].header.frame_id = "world";
  // /* Define a box to add to the world. */
  // shape_msgs::SolidPrimitive primitive2;
  // primitive2.type = primitive.BOX;
  // primitive2.dimensions.resize(3);
  // primitive2.dimensions[0] = 0.01;
  // primitive2.dimensions[1] = 1.5;
  // primitive2.dimensions[2] = 0.01;

  // /* A pose for the box (specified relative to frame_id) */
  // geometry_msgs::Pose box_pose2;
  // box_pose2.orientation.w = 1.0;
  // box_pose2.position.x =  0.55;
  // box_pose2.position.y = 0.0;
  // box_pose2.position.z = -0.08;

  // collision_objects[3].primitives.push_back(primitive2);
  // collision_objects[3].primitive_poses.push_back(box_pose2);
  // collision_objects[3].operation = collision_objects[3].ADD;
  // ROS_INFO("Railing 1 loaded");


//  //Add ur3e_lab as one stl mesh
//   collision_objects[3].id = "ur3e_lab";
//   collision_objects[3].header.frame_id = "world";

//   // Define the primitive and its dimensions. 
//   Vector3d b(1, 1, 1);
//   shapes::Mesh* m1 = shapes::createMeshFromResource("package://ur3e_irl_project/meshes/ur3e_lab.stl"); 
//   ROS_INFO("ur3e lab mesh loaded");
//   shape_msgs::Mesh mesh1;
//   shapes::ShapeMsg mesh_msg1;  
//   shapes::constructMsgFromShape(m1, mesh_msg1);
//   mesh1 = boost::get<shape_msgs::Mesh>(mesh_msg1);
//   collision_objects[3].meshes.resize(1);
//   collision_objects[3].mesh_poses.resize(1);  
//   collision_objects[3].mesh_poses[0].position.x = -0.8;
//   collision_objects[3].mesh_poses[0].position.y = -1.45;
//   collision_objects[3].mesh_poses[0].position.z = -0.9;
//   collision_objects[3].mesh_poses[0].orientation.w= 1.0; 
//   collision_objects[3].mesh_poses[0].orientation.x= 0.0; 
//   collision_objects[3].mesh_poses[0].orientation.y= 0.0;
//   collision_objects[3].mesh_poses[0].orientation.z= 0.0;   
//   collision_objects[3].meshes.push_back(mesh1);
//   collision_objects[3].mesh_poses.push_back(collision_objects[1].mesh_poses[0]);
//   collision_objects[3].operation = collision_objects[1].ADD;
// // ================================================================================================================
  // // Add the kinect camera model.
  // collision_objects[5].id = "kinect_v2";
  // collision_objects[5].header.frame_id = "world";

  // /* Define a box to add to the world. */
  // shape_msgs::SolidPrimitive primitive2;
  // primitive2.type = primitive.BOX;
  // primitive2.dimensions.resize(3);
  // primitive2.dimensions[0] = 0.15;
  // primitive2.dimensions[1] = 0.15;
  // primitive2.dimensions[2] = 0.1;

  // /* A pose for the box (specified relative to frame_id) */
  // geometry_msgs::Pose box_pose2;
  // box_pose2.orientation.w = 0.923879525508;
  // box_pose2.orientation.y = 0.382683449273;
  // box_pose2.position.x =  0.0;
  // box_pose2.position.y = 0.0;
  // box_pose2.position.z = 0.75;

  // collision_objects[5].primitives.push_back(primitive2);
  // collision_objects[5].primitive_poses.push_back(box_pose2);
  // collision_objects[5].operation = collision_objects[5].ADD;
  // ROS_INFO("Kinect_v2 mesh loaded");
  // ros::Duration(1.0).sleep();

  planning_scene_interface.applyCollisionObjects(collision_objects);
  ros::Duration(1.0).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"planning_scene_ur3e");
  ros::NodeHandle nh;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("manipulator");
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
  ros::WallDuration sleep_t(0.5);
  sleep_t.sleep();
  }
  ros::WallDuration(1.0).sleep();  
  
  group.setPlanningTime(60.0);
  group.setPlannerId(group.getDefaultPlannerId(group.getName()));
  addCollisionObjects(planning_scene_interface);
  
  // Wait a bit for ROS things to initialize
  ros::WallDuration(2.0).sleep();
  ROS_INFO("planning_scene node started successfully!");
  ros::waitForShutdown();
  return 0;
}
