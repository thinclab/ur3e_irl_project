// this node will publish the topic "onions_blocks_poses"
// including all current onions blocks, pose is 3-D position

// ros communication:
// subscribe to topic "/current_onions_blocks"
// subscribe to topic "/gazebo/model_states"
// publish the topic "/onions_blocks_poses"

#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Int8MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <ur3e_irl_project/onions_blocks_poses.h>
#include <gazebo_msgs/SetModelState.h>
using namespace std;
// global variables
int g_onions_quantity;
vector<int8_t> g_current_onions_blocks;
vector<double> g_x;
vector<double> g_y;
vector<double> g_z;
bool g_current_onions_callback_started = false;
bool g_onions_poses_updated = false; // act as frequency control of publish loop
bool call_service;
double ur3eRANGE_UPPER_LIMIT, initial_pose_x, initial_pose_y, height_spawning;
ros::ServiceClient setModelState;
gazebo_msgs::SetModelState model_state_srv_msg;
vector<int> indices_deleted;
int goodonionsConvEnd = 0;
int badonionsConvEnd = 0;
int goodonionsInBin = 0;
int badonionsInBin = 0;

string intToString(int a)
{
    stringstream ss;
    ss << a;
    return ss.str();
}

void currentonionsCallback(const std_msgs::Int8MultiArray &current_onions_blocks)
{
    // this topic contains information of what onions blocks have been spawned
    //ROS_INFO_STREAM("\nNow in onions_blocks_poses_publisher. /current_onions_blocks  received from spawner: "<<current_onions_blocks);
    cout << "\nReceived ros msg on /current_onions_blocks in file onions_blocks_poses_publisher.cpp";
    if (!g_current_onions_callback_started)
    {
        // set first time started flag to true
        g_current_onions_callback_started = true;
        ROS_INFO("\nCurrent onions callback has been invoked first time");
    }
    g_onions_quantity = current_onions_blocks.data.size();
    cout << "\nNumber of onions currently available: " << g_onions_quantity << endl;
    g_current_onions_blocks.resize(g_onions_quantity);
    g_current_onions_blocks = current_onions_blocks.data; //This is the color variable we use
                                                          //to differentiate good and bad onions
}

void modelStatesCallback(const gazebo_msgs::ModelStates &current_model_states /*, gazebo_msgs::SetModelState& model_state_srv_msg*/)
{
    // this callback update global values of onions positions
    if (g_current_onions_callback_started)
    {
        // only update when currentonionsCallback has been invoked the first time
        // get onions blocks positions according to settings in g_current_onions_blocks
        vector<double> onions_x;
        vector<double> onions_y;
        vector<double> onions_z;
        onions_x.resize(g_onions_quantity);
        onions_y.resize(g_onions_quantity);
        onions_z.resize(g_onions_quantity);
        // find position of all current onions in topic message
        bool onions_poses_completed = true;
        int ind_del, prev_index = -1;
        for (int i = 0; i < g_onions_quantity; i++)
        {

            // get index of ith onions
            string indexed_model_name;
            if (g_current_onions_blocks[i] == 1)
            { //If color variable is 1, good onion else bad
                indexed_model_name = "onion_" + intToString(i);
            }
            else
            {
                indexed_model_name = "onion_" + intToString(i);
            }
            int index = -1;
            int model_quantity = current_model_states.name.size(); // number of models measured
            for (int j = 0; j < model_quantity; j++)
            {
                if (current_model_states.name[j] == indexed_model_name)
                {
                    index = j;
                    break;
                }
            }
            if (index != -1)
            {
                // this model name exists and has been successfully indexed
                onions_x[i] = current_model_states.pose[index].position.x;
                onions_y[i] = current_model_states.pose[index].position.y;
                onions_z[i] = current_model_states.pose[index].position.z;

                // update global counter for onions in Bin and delete them
                //We're not doing this yet
                // ROS_INFO_STREAM("Index not -1");
                // update global counter for onions reaching end of conveyor
                if (0.65 <= onions_x[i] && onions_x[i] < 0.94)
                {   
                    // ROS_INFO_STREAM("X satisfied");
                    if (0.8 < onions_z[i] && onions_z[i] <= 0.83)
                    {   
                        // ROS_INFO_STREAM("Z satisfied");
                        if (onions_y[i] >= ur3eRANGE_UPPER_LIMIT)
                        {   
                            // ROS_INFO_STREAM("Y satisfied");
                            ROS_INFO_STREAM("Entered conveyor end if block: ");
                            if (g_current_onions_blocks[i] == 0)
                            {
                                goodonionsConvEnd += 1;
                                ROS_INFO_STREAM("Number of good onions that reached the end are: " << goodonionsConvEnd);
                            }
                            else
                            {
                                badonionsConvEnd += 1;
                            }
                            //hard code, or could prompt, or could have command-line arg here:
                            model_state_srv_msg.request.model_state.model_name = indexed_model_name;
                            model_state_srv_msg.request.model_state.pose.position.x = initial_pose_x - 0.1;
                            model_state_srv_msg.request.model_state.pose.position.y = initial_pose_y - i * 0.009; //Just creating a small difference b/w their positions
                            model_state_srv_msg.request.model_state.pose.position.z = height_spawning;
                            setModelState.call(model_state_srv_msg);
                            //make sure service call was successful
                            bool result = model_state_srv_msg.response.success;
                            if (!result)
                                ROS_WARN("service call to set_model_state failed!");
                            else
                            {
                                //Just a comment
                                ROS_INFO_STREAM("Model state changed: " << indexed_model_name);
                            }
                        }
                    }
                }
                if (onions_poses_completed)
                {
                    // only pass data to globals when they are bad
                    g_x = onions_x;
                    g_y = onions_y;
                    g_z = onions_z;
                    // std::cout << "g_x, g_y, g_z updated" << std::endl;
                    if (!g_onions_poses_updated)
                    {
                        // reset flag to true, after updating global value of g_x, g_y, g_z
                        g_onions_poses_updated = true;
                    }
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "onions_blocks_poses_publisher");
    ros::NodeHandle nh;

    // initialize subscribers for "/current_onions_blocks" and "/gazebo/model_states"
    ros::Subscriber current_onions_subscriber = nh.subscribe("/current_onions_blocks", 1, currentonionsCallback);
    setModelState = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    //Leave the next line for educational purposes
    //ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, boost::bind(modelStatesCallback, _1, model_state_srv_msg));
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    // initialize publisher for "/onions_blocks_poses"
    ros::Publisher onions_poses_publisher = nh.advertise<ur3e_irl_project::onions_blocks_poses>("onions_blocks_poses", 1);
    ur3e_irl_project::onions_blocks_poses current_poses_msg;
    cout << "\ng_onions_poses_updated: " << g_onions_poses_updated;
    nh.getParam("/ur3eRANGE_UPPER_LIMIT", ur3eRANGE_UPPER_LIMIT);
    nh.getParam("/initial_pose_x", initial_pose_x);
    nh.getParam("/initial_pose_y", initial_pose_y);
    nh.getParam("/height_spawning", height_spawning);
    // publishing loop
    while (ros::ok())
    {
        if (g_onions_poses_updated)
        {
            // only publish when onions positions are updated
            // no need to publish repeated data
            g_onions_poses_updated = false; // set flag to falsbade
            // there is tiny possibility that g_x is not in the length of g_onions_quantity
            int local_onions_quantity = g_x.size(); // so get length of g_x
            current_poses_msg.x.resize(local_onions_quantity);
            current_poses_msg.y.resize(local_onions_quantity);
            current_poses_msg.z.resize(local_onions_quantity);
            current_poses_msg.x = g_x;
            current_poses_msg.y = g_y;
            current_poses_msg.z = g_z;
            //ROS_INFO_STREAM("Onion current pose: "<<current_poses_msg);
            onions_poses_publisher.publish(current_poses_msg);
        }
        // rate_timer.sleep();
        ros::spinOnce();
    }
    return 0;
}
