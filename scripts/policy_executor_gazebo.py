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
import numpy as np

''' Picked/AtHome - means ur3e is in hover plane at home position
    onionLoc = {0: 'OnConveyor', 1: 'InFront',
        2: 'InBin', 3: 'Picked/AtHome'}
    eefLoc = {0: 'OnConveyor', 1: 'InFront', 2: 'InBin', 3: 'Picked/AtHome'}
    predictions = {0: 'Bad', 1: 'Good', 2: 'Unknown'}
    listIDstatus = {0: 'Empty', 1: 'Not Empty', 2: 'Unavailable'}
    actList = {0: 'InspectAfterPicking', 1: 'PlaceOnConveyor', 2: 'PlaceInBin', 3: 'Pick',
        4: 'ClaimNewOnion', 5: 'InspectWithoutPicking', 6: 'ClaimNextInList'} '''

# Global initializations
flag = False
good_onion = False
pnp = PickAndPlace()
idx = -1
policy = np.genfromtxt('/home/psuresh/catkin_ws/src/ur3e_irl_project/scripts/learned_policy.csv', delimiter=' ')
# policy = np.genfromtxt('/home/psuresh/catkin_ws/src/ur3e_irl_project/scripts/expert_policy.csv', delimiter=' ')
# policy = np.genfromtxt('/home/psuresh/catkin_ws/src/ur3e_irl_project/scripts/test_expert_policy.csv', delimiter=' ')

def sid2vals(s, nOnionLoc=4, nEEFLoc=4, nPredict=3, nlistIDStatus=3):
    sid = s
    onionloc = int(mod(sid, nOnionLoc))
    sid = (sid - onionloc)/nOnionLoc
    eefloc = int(mod(sid, nEEFLoc))
    sid = (sid - eefloc)/nEEFLoc
    predic = int(mod(sid, nPredict))
    sid = (sid - predic)/nPredict
    listidstatus = int(mod(sid, nlistIDStatus))
    return [onionloc, eefloc, predic, listidstatus]


def vals2sid(ol, eefl, pred, listst, nOnionLoc=5, nEEFLoc=4, nPredict=3, nlistIDStatus=3):
    return(ol + nOnionLoc * (eefl + nEEFLoc * (pred + nPredict * listst)))


def getState(onionName, predic):
    global pnp
    print("Getting state")
    current_pose = pnp.group.get_current_pose().pose
    if current_pose.position.x > 0.5 and current_pose.position.x < 0.9 and current_pose.position.y > -0.5 and current_pose.position.y < 0.5 and current_pose.position.z > 0 and current_pose.position.z < 0.2:
        pnp.eefLoc = 0  # On Conveyor
    elif current_pose.position.x > 0.4 and current_pose.position.x < 0.5 and current_pose.position.y > -0.1 and current_pose.position.y < 0.15 and current_pose.position.z > 0.44 and current_pose.position.z < 0.6:
        pnp.eefLoc = 1  # In Front
    elif current_pose.position.x > 0 and current_pose.position.x < 0.15 and current_pose.position.y > 0.5 and current_pose.position.y < 0.8:
        pnp.eefLoc = 2  # In Bin
    elif current_pose.position.x > 0.45 and current_pose.position.x < 0.9 and current_pose.position.y > -0.5 and current_pose.position.y < 0.5 and current_pose.position.z > 0.15 and current_pose.position.z < 0.5:
        pnp.eefLoc = 3  # In the Hover plane
    else:
        print("Couldn't find valid eef state!")
        print("EEF coordinates: ", current_pose)
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        model_coordinates = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        print("Onion name is: ", onionName)
        onion_coordinates = model_coordinates(onionName, "").pose
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    if onion_coordinates.position.x > 0.5 and onion_coordinates.position.x < 0.9 and onion_coordinates.position.y > -0.6 and onion_coordinates.position.y < 0.3 and onion_coordinates.position.z > 0.8 and onion_coordinates.position.z < 0.9:
        pnp.onionLoc =  0    # On Conveyor
        print("OnionLoc is: On conveyor {0}".format(pnp.onionLoc))
    elif onion_coordinates.position.x > 0.15 and onion_coordinates.position.x < 0.3 and onion_coordinates.position.z > 1 and onion_coordinates.position.z < 2:
        pnp.onionLoc =  1    # In Front
        print("OnionLoc is: In Front {0}".format(pnp.onionLoc))
    elif onion_coordinates.position.x > 0 and onion_coordinates.position.x < 0.15 and onion_coordinates.position.y > 0.5 and onion_coordinates.position.y < 0.8:
        pnp.onionLoc =  2    # In Bin
        print("OnionLoc is: In Bin {0}".format(pnp.onionLoc))
    elif onion_coordinates.position.x > 0.35 and onion_coordinates.position.x < 0.9 and onion_coordinates.position.z > 0.9 and onion_coordinates.position.z < 1.5:
        pnp.onionLoc =  3    # In Hover Plane
        print("OnionLoc is: In Hover plane {0}".format(pnp.onionLoc))
    elif onion_coordinates.position.x > 0.5 and onion_coordinates.position.x < 0.9 and onion_coordinates.position.y > 0.3 and onion_coordinates.position.y < 0.6 and onion_coordinates.position.z > 0.8 and onion_coordinates.position.z < 0.9:
        pnp.onionLoc =  0    # Placed on Conveyor   # we removed the placed on conv location
        print("OnionLoc is: Placed on conveyor {0}".format(pnp.onionLoc))
    else:
        print("Couldn't find valid onion state!")
        print("Onion coordinates: ", onion_coordinates)

    pnp.prediction = predic
    print("EEfloc is: ", pnp.eefLoc)
    print("prediction is: ", predic)
    if len(pnp.bad_onions) > 0:
        pnp.listIDstatus = 1
    elif len(pnp.bad_onions) == 0:
        pnp.listIDstatus = 0
    else:
        pnp.listIDstatus = 2
    print("List status is: ", pnp.listIDstatus)

    return vals2sid(ol=pnp.onionLoc, eefl=pnp.eefLoc, pred=pnp.prediction, listst=pnp.listIDstatus)


def executePolicyAct(action, onionName, attach_srv, detach_srv, max_index):
    global pnp, flag, idx
    if action == 0:     # Inspect after picking
        print("Inspect after picking")
        pnp.view(0.3)
        rospy.sleep(0.01)
        pnp.rotategripper(0.3)
        if pnp.listIDstatus == 2:
            flag = True
    elif action == 1:   # Place on conveyor
        print("Place on conveyor")
        pnp.placeOnConveyor()
        rospy.sleep(0.01)
        detach_srv.call(pnp.req)
        # pnp.liftgripper()
        pnp.num_onions = pnp.num_onions - 1
        if pnp.listIDstatus == 2:
            flag = False
    elif action == 2:   # Place in bin
        print("Place in bin")
        pnp.goto_bin()
        rospy.sleep(0.01)
        detach_srv.call(pnp.req)
        pnp.num_onions = pnp.num_onions - 1
        if pnp.listIDstatus == 2:
            flag = False
    elif action == 3:   # Pick
        print("Pick")
        pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
        status = pnp.waitToPick()
        if(status):
            attach_srv.call(pnp.req)
            rospy.sleep(0.01)
            pnp.liftgripper()
            rospy.sleep(0.01)
    elif action == 4:   # Claim new onion
        print("Claim new onion")
        if (pnp.onion_index == max_index):
            print("Onion index is: ", pnp.onion_index)
            pnp.onion_index = -1
            pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
            print("Reached the end of onion list")
            rospy.signal_shutdown("Shutting down node, work is done")
        else:
            pnp.onion_index = pnp.onion_index + 1
            print("Updated onion index is:", pnp.onion_index)
    elif action == 5:   # Inspect without picking
        print("Inspect without picking")
        if not flag:
            print "goto_home()"
            pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
            rospy.sleep(0.01)
            pnp.roll(0.3)
            print "Going back home now"
            pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
            flag = True
    else:   # Claim next in list
        print("Claim next in list")
        if (pnp.onion_index == pnp.bad_onions[-1]):
            print("Onion index is: ", pnp.onion_index)
            pnp.onion_index = -1
            pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
            print("Reached the end of onion list")
            rospy.signal_shutdown("Shutting down node, work is done")
        else:
            idx += 1
            pnp.onion_index = pnp.bad_onions[idx]
            print("Updated onion index is:", pnp.onion_index)
    return


def callback_poses(onions_poses_msg):
    global pnp
    if(pnp.onion_index == -1):
        print("No more onions to sort!")
        rospy.signal_shutdown("Shutting down node, work is done")
    else:
        if(pnp.onion_index == len(onions_poses_msg.x)):
            return
        else:
            # print("Setting target coordinates for onion_index = ",pnp.onion_index)
            current_onions_x = onions_poses_msg.x
            current_onions_y = onions_poses_msg.y
            current_onions_z = onions_poses_msg.z
            pnp.target_location_x = current_onions_x[pnp.onion_index]
            pnp.target_location_y = current_onions_y[pnp.onion_index]
            pnp.target_location_z = current_onions_z[pnp.onion_index]
    # print "target_location_x,target_location_y"+str((pnp.target_location_x,pnp.target_location_y))
    return


def callback_exec_policy(color_indices_msg):
    global pnp, policy, good_onion
    max_index = len(color_indices_msg.data)
    pnp.bad_onions = []
    if (color_indices_msg.data[pnp.onion_index] == 1):
        pnp.req.model_name_1 = "onion_" + str(pnp.onion_index)
        print "Onion name set in IF as: ", pnp.req.model_name_1
        if(pnp.onion_index is max_index):
            pnp.onion_index = -1
            rospy.signal_shutdown("Shutting down node, work is done")
    else:
        if pnp.onion_index not in pnp.bad_onions:
            pnp.onion_index += 1
            return
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
    pnp.num_onions = len(color_indices_msg.data)

    if(pnp.onion_index != max_index):

        print "(model_1,link_1,model_2,link_2)", pnp.req.model_name_1, pnp.req.link_name_1, pnp.req.model_name_2, pnp.req.link_name_2
        print("Sending onion name: {}, prediction: {} to state check".format(
            pnp.req.model_name_1, color_indices_msg.data[pnp.onion_index]))
        if flag:
            pnp.bad_onions = [i for i in range(pnp.onion_index, max_index) if (color_indices_msg.data[i] == 0)]
            print("Bad onion indices are: ", pnp.bad_onions)
            s = getState(pnp.req.model_name_1,
                         color_indices_msg.data[pnp.onion_index])
        else:
            s = getState(pnp.req.model_name_1, 2)
        print("State id: ", s)
        a = policy[s]
        executePolicyAct(a, pnp.req.model_name_1,
                         attach_srv, detach_srv, max_index)
    else:
        print("Finished considering all onions. Stopping node!")
        rospy.signal_shutdown("Shutting down node, work is done")

def main():
    global pnp
    print "goto_home()"
    pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
    try:
        rospy.Subscriber("current_onions_blocks",
                         Int8MultiArray, callback_exec_policy)
        rospy.Subscriber("onions_blocks_poses",
                         onions_blocks_poses, callback_poses)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "Main function not found! WTH dude!"