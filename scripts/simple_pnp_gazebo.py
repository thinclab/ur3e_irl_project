#!/usr/bin/env python

from PickandPlace import PickAndPlace

import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Int8MultiArray
from operator import mod
import random
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import sys
import rospy
from ur3e_irl_project.msg import onions_blocks_poses
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# Global initializations
pnp = PickAndPlace()
flag = False
good_onion = False


def callback_poses(onions_poses_msg):
    global pnp, good_onion
    onion_index = pnp.onion_index
    if good_onion:
        # print("I'm waiting for a bad onion!")
        return
    if(onion_index == -1):
        print("No more onions to sort!")
        rospy.signal_shutdown("Shutting down node, work is done")
    else:
        if(onion_index == len(onions_poses_msg.x)):
            rospy.signal_shutdown("Shutting down node, work is done")
        else:
            current_onions_x = onions_poses_msg.x
            current_onions_y = onions_poses_msg.y
            current_onions_z = onions_poses_msg.z
            pnp.target_location_x = current_onions_x[onion_index]
            pnp.target_location_y = current_onions_y[onion_index]
            pnp.target_location_z = current_onions_z[onion_index]
    # print "target_location_x,target_location_y"+str((target_location_x,target_location_y))
    return


def callback_onion_pick(color_indices_msg):
    global flag, pnp, good_onion
    max_index = len(color_indices_msg.data)
    if (color_indices_msg.data[pnp.onion_index] == 1):
        good_onion = True
        pnp.req.model_name_1 = "onion_" + str(pnp.onion_index)
        print "Onion name set in IF as: ", pnp.req.model_name_1
        if(pnp.onion_index is not max_index - 1):
            pnp.onion_index = pnp.onion_index + 1
        else:
            pnp.onion_index = -1
            rospy.signal_shutdown("Shutting down node, work is done")
        return
    else:
        good_onion = False
        pnp.req.model_name_1 = "onion_" + str(pnp.onion_index)
        print "Onion name set in ELSE as: ", pnp.req.model_name_1

    # attach and detach service
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()
    pnp.req.link_name_1 = "base_link"
    pnp.req.model_name_2 = "ur3e"
    pnp.req.link_name_2 = "right_l6"
    if not flag:
        pnp.num_onions = len(color_indices_msg.data)
        flag = True

    if(pnp.num_onions > 0):

        print "(model_1,link_1,model_2,link_2)", pnp.req.model_name_1, pnp.req.link_name_1, pnp.req.model_name_2, pnp.req.link_name_2
        ##############################################
        print "goto_home()"
        pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)

        status = pnp.waitToPick()
        # print "status: ", status
        # status = False
        if(status):
            attach_srv.call(pnp.req)
            rospy.sleep(0.01)
            pnp.liftgripper()
            rospy.sleep(0.01)
            # current_pose = pnp.group.get_current_pose().pose
            # print "current_pose: " + str((current_pose))
            # pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
            # current_pose = pnp.group.get_current_pose().pose
            # print "current_pose: " + str((current_pose))
            # rospy.sleep(50000000)
            pnp.view(0.3)
            rospy.sleep(0.01)
            pnp.rotategripper(0.3)
            rospy.sleep(0.01)
            if (color_indices_msg.data[pnp.onion_index] == 1):
                pnp.placeOnConveyor()
            else:
                pnp.goto_bin()
            rospy.sleep(0.01)
            detach_srv.call(pnp.req)
            pnp.num_onions = pnp.num_onions - 1
            if (pnp.onion_index == max_index - 1):
                print("Onion index is: ", pnp.onion_index)
                pnp.onion_index = -1
                pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
                print("Reached the end of onion list")
                rospy.signal_shutdown("Shutting down node, work is done")
            else:
                pnp.onion_index = pnp.onion_index + 1
                print("Updated onion index is:", pnp.onion_index)
        ##############################################
    else:
        print("Num onions is zero!")
        rospy.signal_shutdown("Shutting down node, work is done")


def callback_onion_roll(color_indices_msg):
    global flag, pnp, good_onion
    max_index = len(color_indices_msg.data)

    if (color_indices_msg.data[pnp.onion_index] == 1):
        good_onion = True
        pnp.req.model_name_1 = "onion_" + str(pnp.onion_index)
        print "Onion name set in IF as: ", pnp.req.model_name_1
        if(pnp.onion_index is not max_index - 1):
            pnp.onion_index = pnp.onion_index + 1
        else:
            pnp.onion_index = -1
            rospy.signal_shutdown("Shutting down node, work is done")
        return
    else:
        good_onion = False
        pnp.req.model_name_1 = "onion_" + str(pnp.onion_index)
        print "Onion name set in ELSE as: ", pnp.req.model_name_1

    # attach and detach service
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()
    pnp.req.link_name_1 = "base_link"
    pnp.req.model_name_2 = "ur3e"
    pnp.req.link_name_2 = "right_l6"
    if not flag:
        pnp.num_onions = len(color_indices_msg.data)
        print "goto_home()"
        pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
        rospy.sleep(0.01)
        roll = pnp.roll(0.3)
        flag = True
    if(pnp.num_onions > 0):

        pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
        status = pnp.waitToPick()
        # print "status: ", status
        # status = False
        if(status):
            attach_srv.call(pnp.req)
            rospy.sleep(0.01)
            pnp.liftgripper()
            rospy.sleep(0.01)
            pnp.goto_bin()
            rospy.sleep(0.01)
            detach_srv.call(pnp.req)
            pnp.num_onions = pnp.num_onions - 1
            if (pnp.onion_index == max_index - 1):
                print("Onion index is:", pnp.onion_index)
                pnp.onion_index = -1
                pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
                print("Reached the end of onion list.")
                rospy.signal_shutdown("Shutting down node, work is done")
            else:
                pnp.onion_index = pnp.onion_index + 1
                print("Updated onion index is:", pnp.onion_index)
        ##############################################
    else:
        print("Num onions is zero!")
        return


##################################### Now to Main ##################################################

def main():
    print("I'm in main!")
    try:
        # if len(sys.argv) < 2:
        #     sortmethod = "pick"   # Default sort method
        # else:
        #     sortmethod = sys.argv[1]
        # if(sortmethod == "pick"):
        #     print("Pick method selected")
        #     rospy.Subscriber("current_onions_blocks",
        #                      Int8MultiArray, callback_onion_pick)
        # elif(sortmethod == "roll"):
        #     print("Roll method selected")
        #     rospy.Subscriber("current_onions_blocks",
        #                      Int8MultiArray, callback_onion_roll)
        #     # pass
        # rospy.Subscriber("onions_blocks_poses",
        #                  onions_blocks_poses, callback_poses)
        # #########################################################################################
        # # callback_onion_pick(rospy.wait_for_message("current_onions_blocks", Int8MultiArray))  #
        # # If you're using this method, it will onpnp.req.model_name_1 = Nonely listen once until it hears something,      #
        # # this may cause trouble later, watch out!                                            #
        # #######################################################################################

        # pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
  
        group = pnp.group
        current_pose = group.get_current_pose().pose
        allow_replanning = False
        planning_time = 5
        status = pnp.go_to_pose_goal(pnp.q[0], pnp.q[1], pnp.q[2], pnp.q[3], 0.75, 0.0, 0.2, allow_replanning, planning_time, thresh = 0.001)
        rospy.sleep(0.1)
        print "\n",group.get_current_pose().pose.position


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    rospy.spin()


if __name__ == '__main__':
    print("Calling Main!")
    try:
        main()
    except rospy.ROSInterruptException:
        print "Main function not found! WTH dude!"
