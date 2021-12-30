// Author: Prasanth Suresh(ps32611@uga.edu);
// Description: We  have good onions and bad onions.
// Aim of this script is to spawn them on the conveyor surface.

// STILL UNDER CONSTRUCTION!
// Do not edit/copy without permission.

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <gazebo_msgs/SpawnModel.h>
#include <std_msgs/Int8MultiArray.h>
#include <ur3e_irl_project/StringArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Int32.h"

using namespace std;
// int to string converter
string intToString(int a)
{
    stringstream ss;
    ss << a;
    return ss.str();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "onion_blocks_spawner");
    ros::NodeHandle nh; //("~");
    int i = 0 /*index the onions*/, j = 0 /*position reference for onions*/;
    int onion_gen = 0, total_onions = 4;
    double initial_pose_x, initial_pose_y, height_spawning, spawning_interval,
        conveyor_center_x, belt_width, wrench_duration, randpos, object_width;
    bool spawn_multiple;
    string good_onion_path, bad_onion_path, good_xmlStr, bad_xmlStr, model_name;
    int8_t color_index; // 0 is bad, 1 is good
    ifstream good_inXml, bad_inXml;
    stringstream good_strStream, bad_strStream;
    nh.getParam("/initial_pose_x", initial_pose_x);
    nh.getParam("/initial_pose_y", initial_pose_y);
    nh.getParam("/height_spawning", height_spawning);
    nh.getParam("/spawning_interval", spawning_interval);
    nh.getParam("/conveyor_center_x", conveyor_center_x);
    nh.getParam("/belt_width", belt_width);
    nh.getParam("/spawn_multiple", spawn_multiple);
    nh.getParam("/object_width", object_width);
    // get file path of onions from parameter server
    bool get_good_onion_path, get_bad_onion_path;
    get_good_onion_path = nh.getParam("/good_onion_path", good_onion_path);
    get_bad_onion_path = nh.getParam("/bad_onion_path", bad_onion_path);

    ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel spawn_model_srv_msg; // service message

    // make sure /gazebo/spawn_sdf_model service is ready
    bool service_ready = false;
    while (!service_ready)
    {
        service_ready = ros::service::exists("/gazebo/spawn_sdf_model", true);
        ROS_INFO("waiting for spawn_sdf_model service");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("spawn_sdf_model service is ready");

    if (!(get_good_onion_path && get_bad_onion_path))
    {
        ROS_INFO("failed to get parameters");
        return 0; // return if fail to get parameters
    }

    // prepare the xml for service call, read sdf into string
    // good onion
    good_inXml.open(good_onion_path.c_str());
    good_strStream << good_inXml.rdbuf();
    good_xmlStr = good_strStream.str();
    // bad onion
    bad_inXml.open(bad_onion_path.c_str());
    bad_strStream << bad_inXml.rdbuf();
    bad_xmlStr = bad_strStream.str();
    // prepare the spawn model service message
    spawn_model_srv_msg.request.initial_pose.position.x = initial_pose_x;  // 4.0
    spawn_model_srv_msg.request.initial_pose.position.y = initial_pose_y;  // 4.0
    spawn_model_srv_msg.request.initial_pose.position.z = height_spawning; // 0.2 ; on the conveyor belt
    spawn_model_srv_msg.request.initial_pose.orientation.x = 0.7071067;    // Euler rotation of 90 deg about x axis
    spawn_model_srv_msg.request.initial_pose.orientation.y = 0.0;          // converted to quaternion.
    spawn_model_srv_msg.request.initial_pose.orientation.z = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.w = 0.7071069;
    spawn_model_srv_msg.request.reference_frame = "world";

    //begin spawning onions

    // publisher for /modelnames
    ros::Publisher modelnames_publisher = nh.advertise<ur3e_irl_project::StringArray>("modelnames", 1);

    ros::Publisher current_onions_publisher = nh.advertise<std_msgs::Int8MultiArray>("current_onions_blocks", 1);

    ur3e_irl_project::StringArray modelnames_msg;
    modelnames_msg.modelnames.clear();
    // publisher for /current_onions
    std_msgs::Int8MultiArray current_onions_msg;
    current_onions_msg.data.clear();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(total_onions);

    ros::Duration(1).sleep();
    while (ros::ok())
    {

        while (i < total_onions)
        {
            string index_string = intToString(i);
            ++j;
            if (i % 2 == 0)
            {
                spawn_model_srv_msg.request.initial_pose.position.x = initial_pose_x + j * 0.1; //width of sphere is 0.02, well within 0.05
                spawn_model_srv_msg.request.initial_pose.position.y = initial_pose_y - j * 0.05; //width of sphere is 0.02, well within 0.05
            }
            else
            {
                spawn_model_srv_msg.request.initial_pose.position.x = initial_pose_x - j * 0.1;
                spawn_model_srv_msg.request.initial_pose.position.y = initial_pose_y - j * 0.05;
            }
            ROS_INFO_STREAM("x position of new onion: "
                            << spawn_model_srv_msg.request.initial_pose.position.x);
            ROS_INFO_STREAM("y position of new onion: "
                            << spawn_model_srv_msg.request.initial_pose.position.y);

            /* initialize random seed: */
            // srand (time(NULL));

            /* generate random number between 1 and 10: */
            //onion_gen = std::rand() % 100;
            onion_gen = i;

            if ((onion_gen % 2) == 0)
            {
                cout << "Generating good onion";
                color_index = 1;
                model_name = "onion_" + index_string; // initialize model_name
                spawn_model_srv_msg.request.model_name = model_name;
                spawn_model_srv_msg.request.robot_namespace = "onion_" + index_string;
                spawn_model_srv_msg.request.model_xml = good_xmlStr;
            }

            else
            {
                cout << "Generating bad onion";
                color_index = 0;
                model_name = "onion_" + index_string; // initialize model_name
                spawn_model_srv_msg.request.model_name = model_name;
                spawn_model_srv_msg.request.robot_namespace = "onion_" + index_string;
                spawn_model_srv_msg.request.model_xml = bad_xmlStr;
            }

            collision_objects[i].id = model_name;
            collision_objects[i].header.frame_id = "world";
            /* Define a box to add to the world. */
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.052;
            primitive.dimensions[1] = 0.052;
            primitive.dimensions[2] = 0.052;

            /* A pose for the box (specified relative to frame_id) */
            geometry_msgs::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x =  spawn_model_srv_msg.request.initial_pose.position.x;
            box_pose.position.y = spawn_model_srv_msg.request.initial_pose.position.y;
            box_pose.position.z = -0.09;

            collision_objects[i].primitives.push_back(primitive);
            collision_objects[i].primitive_poses.push_back(box_pose);
            collision_objects[i].operation = collision_objects[i].ADD;

            // call spawn model service
            bool call_service = spawn_model_client.call(spawn_model_srv_msg);
            if (call_service)
            {
                if (spawn_model_srv_msg.response.success)
                {
                    ROS_INFO_STREAM(model_name << " has been spawned");
                }
                else
                {
                    ROS_INFO_STREAM(model_name << " spawn failed");
                }
            }
            else
            {
                ROS_INFO("fail in first call");
                ROS_ERROR("fail to connect with gazebo server");
                return 0;
            }
            // publish current onion blocks status, all onion blocks will be published
            // no matter if it's successfully spawned, or successfully initialized in speed
            current_onions_msg.data.push_back(color_index);

            //ROS_INFO_STREAM("Current onion poses: "<< current_onions_msg);
            modelnames_msg.modelnames.push_back(spawn_model_srv_msg.request.model_name);

            // loop end, increase index by 1
            i++;
        }
        planning_scene_interface.applyCollisionObjects(collision_objects);
        ros::Duration(0.1).sleep();
        current_onions_publisher.publish(current_onions_msg);
        modelnames_publisher.publish(modelnames_msg);
        //ros::Duration(5).sleep();
        ROS_INFO_STREAM("Current onion color indices" << current_onions_msg); //Debug print statement
        ROS_INFO_STREAM("Current Model names" << modelnames_msg);             //Debug print statement
        ros::spinOnce();
        ros::Duration(spawning_interval).sleep(); // frequency control, spawn one onion in each loop
        // delay time decides density of the onions
    }

    return 0;
}