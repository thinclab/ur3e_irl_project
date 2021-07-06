#!/usr/bin/env python

from PickandPlace import PickAndPlace
from gripper_to_position import reset_gripper, activate_gripper, gripper_to_pos

import numpy as np
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from rospy.client import init_node
from std_msgs.msg import String, Int8MultiArray
from operator import mod
import random
import sys
from os import system
import rospy
from time import sleep
from ur3e_irl_project.msg import OBlobs
from sanet_onionsorting.srv import yolo_srv
from smach import *
from smach_ros import *
from smach_msgs.msg import *
import rospkg


rospack = rospkg.RosPack()  # get an instance of RosPack with the default search paths
# get the file path for sanet_onionsorting
path = rospack.get_path('sanet_onionsorting')
sys.path.append(path + '/scripts/')

# from rgbd_imgpoint_to_tf import Camera

# Global initializations
pnp = PickAndPlace(init_node=False)
current_state = 140
done_onions = 0
# camera = None 


# def getCameraInstance(camInstance):
#     global camera
#     camera = camInstance
#     return

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


def vals2sid(ol, eefl, pred, listst, nOnionLoc=4, nEEFLoc=4, nPredict=3, nlistIDStatus=3):
    return(ol + nOnionLoc * (eefl + nEEFLoc * (pred + nPredict * listst)))


class Get_info(State):
    def __init__(self):
        # global camera
        State.__init__(self, outcomes=['updated', 'not_updated', 'timed_out', 'completed'],
                       input_keys=['x', 'y', 'z', 'color', 'counter'],
                       output_keys=['x', 'y', 'z', 'color', 'counter'])
        self.x = []
        self.y = []
        self.z = []
        self.color = []
        self.is_updated = False
        # rospy.wait_for_service("/get_predictions")  # Contains the centroids of the obj bounding boxes
        # gip_service = rospy.ServiceProxy("/get_predictions", yolo_srv)
        # response = gip_service()
        # camera.save_response(response)
        # print "\nIs updated: ",camera.is_updated,"\tFound objects: ", camera.found_objects
        # if camera.is_updated and camera.found_objects:    
        #     camera.OblobsPublisher()
        self.callback_vision(rospy.wait_for_message("/object_location", OBlobs))

    def callback_vision(self, msg):
        # print '\nCallback vision\n'
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.color = msg.color
        # print '\nCallback data x,y,z in Get_info are: \n', self.x,self.y,self.z
        # rospy.sleep(5)
        self.is_updated = True
        return

    def execute(self, userdata):
        # rospy.loginfo('Executing state: Get_info')
        if userdata.counter >= 500:
            userdata.counter = 0
            return 'timed_out'

        if self.is_updated == True:
            if len(self.x) > 0 and self.x[0] != -100:
                userdata.x = self.x
                userdata.y = self.y
                userdata.z = self.z
                userdata.color = self.color
                userdata.counter = 0
                rospy.sleep(0.01)
                # print("I'm updated")
                # print '\nUser data x,y,z in Get_info are: \n', userdata.x,userdata.y,userdata.z
                # rospy.sleep(5)
                return 'updated'
            else:
                print '\nSort Complete!\n'
                return 'completed'
        else:
            userdata.counter += 1
            # print("I'm not updated")
            return 'not_updated'


class Claim(State):
    def __init__(self):
        State.__init__(self, outcomes=['updated', 'not_updated', 'timed_out', 'not_found', 'completed'],
                       input_keys=['x', 'y', 'z', 'color', 'counter'],
                       output_keys=['x', 'y', 'z', 'color', 'counter'])
        self.is_updated = False
        # print("I came to claim")

    def execute(self, userdata):
        global pnp, done_onions, current_state
        # rospy.loginfo('Executing state: Claim')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        if len(userdata.color) == 0:
            return 'not_found'
        max_index = len(userdata.color)
        print '\nMax index is = ', max_index
        pnp.onion_index = 0
        for i in range(max_index):
            if len(userdata.y) >= i:
                try:
                    if userdata.x[i] > -0.35 and userdata.x[i] < 0.35:
                        pnp.target_location_x = userdata.x[i]
                        pnp.target_location_y = userdata.y[i]
                        pnp.target_location_z = userdata.z[i]
                        pnp.onion_color = userdata.color[i]
                        pnp.onion_index = i
                        self.is_updated = True
                        break
                    else:
                        done_onions += 1
                        print '\nDone onions = ', done_onions

                except IndexError:
                    pass
            else:
                print '\nSort complete!'
                return 'completed'

        if max_index == done_onions:
            print '\nAll onions are sorted!'
            return 'completed'
        else:
            done_onions = 0

        if self.is_updated == False:
            userdata.counter += 1
            return 'not_updated'
        else:
            print '\n(x,y,z) after claim: ',pnp.target_location_x,pnp.target_location_y,pnp.target_location_z
            reset_gripper()
            activate_gripper()
            userdata.counter = 0
            current_state = vals2sid(ol=0, eefl=3, pred=2, listst=2)
            rospy.sleep(0.05)
            return 'updated'


class Approach(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                       input_keys=['x', 'y', 'z', 'color', 'counter'],
                       output_keys=['x', 'y', 'z', 'color', 'counter'])

    def execute(self, userdata):
        global pnp
        # rospy.loginfo('Executing state: Approach')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        # home = pnp.goto_home(tolerance=0.1, goal_tol=0.1, orientation_tol=0.1)
        home = True
        gripper_to_pos(50, 60, 200, False)    # GRIPPER TO POSITION 0
        rospy.sleep(0.1)
        if home:
            status = pnp.goAndPick()
            rospy.sleep(0.1)
            if status:
                userdata.counter = 0
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'
        else:
            userdata.counter += 1
            return 'failed'


class Dipdown(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                       input_keys=['x', 'y', 'z', 'color', 'counter'],
                       output_keys=['x', 'y', 'z', 'color', 'counter'])

    def execute(self, userdata):
        global pnp
        # rospy.loginfo('Executing state: Dipdown')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        dip = pnp.staticDip(z_pose=1.35)
        rospy.sleep(0.1)
        if dip:
            userdata.counter = 0
            return 'success'
        else:
            userdata.counter += 1
            return 'failed'


class Grasp_object(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                       input_keys=['x', 'y', 'z', 'color', 'counter'],
                       output_keys=['x', 'y', 'z', 'color', 'counter'])

    def execute(self, userdata):
        global pnp, current_state
        # rospy.loginfo('Executing state: Grasp_object')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        gr = gripper_to_pos(255, 60, 200, False)    # GRIPPER TO POSITION 60
        rospy.sleep(2)
        if gr:
            userdata.counter = 0
            current_state = vals2sid(ol=3, eefl=3, pred=2, listst=2)
            return 'success'
        else:
            userdata.counter += 1
            return 'failed'


class Liftup(State):
    def __init__(self):
        self.grasp = True
        State.__init__(self, outcomes=['success', 'failed', 'no_grasp', 'timed_out'],
                       input_keys=['counter'],
                       output_keys=['counter'])

    def execute(self, userdata):
        global pnp
        # rospy.loginfo('Executing state: Pick')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'
        else:
            lift = pnp.liftgripper()
            rospy.sleep(0.05)
            if lift:
                self.callback_graspCheck(rospy.wait_for_message("/object_location", OBlobs))
                if self.grasp == False:
                    return 'no_grasp'
                else:
                    print "\nSuccessfully grasped and lifted"
                    userdata.counter = 0
                    '''NOTE: Both place on conveyor and pick use this, so don't update current state here.'''
                    return 'success'
            else:
                userdata.counter += 10.0363871693106,0.381861362699,1.60594109992
                return 'failed'

    def callback_graspCheck(self, msg):
        for i in range(len(msg.x)):
            if pnp.target_location_z - 0.05 <= msg.z[i] <= pnp.target_location_z + 0.05:
                print "\nZ value that matched: ", msg.z[i]
                if pnp.target_location_x - 0.05 <= msg.x[i] <= pnp.target_location_x + 0.05:
                    print "\nX value that matched: ",msg.x[i] 
                    if pnp.target_location_y - 0.05 <= msg.y[i] <= pnp.target_location_y + 0.05:
                        print "\nY value that matched: ",msg.y[i]
                        # rospy.sleep(20)
                        self.grasp = False
                    else: print "\nY value didn't match"
                else: print "\nX value didn't match"


class View(State):
    def __init__(self):
        self.color = None
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                       input_keys=['color', 'counter'],
                       output_keys=['color', 'counter'])

    def execute(self, userdata):
        global pnp, current_state
        # rospy.loginfo('Executing state: View')
        if userdata.counter >= 2:
            userdata.counter = 0
            return 'timed_out'
        
        if pnp.onion_color == 1:    # Inspect further only if it is an unblemished one
            print "\nChecking color before rotation"
            self.checkOnionColor()
            rotate = pnp.rotategripper(0.3)
            rospy.sleep(1)
            if rotate:
                print "\nSuccessfully Rotated!"
                self.checkOnionColor()
                if self.color != None:
                    current_state = int(vals2sid(ol=1, eefl=1, pred=self.color, listst=2))
                    print "\nCurrent state is: ", current_state
                    # rospy.sleep(100)
                    userdata.counter = 0
                    return 'success'
                else:
                    # print("\nCurrent repeat count is: ",userdata.counter)
                    userdata.counter += 1
                    return 'failed'
            else:
                userdata.counter += 1
                return 'failed'
        else:                   # Otherwise, directly go to the bin
            self.color = int(pnp.onion_color)
            if self.color != None:
                current_state = int(vals2sid(ol=1, eefl=1, pred=self.color, listst=2))
                print "\nCurrent state is: ", current_state
                # rospy.sleep(100)
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'

    def checkOnionColor(self):
        # global camera
        # rospy.wait_for_service("/get_predictions")  # Contains the centroids of the obj bounding boxes
        # gip_service = rospy.ServiceProxy("/get_predictions", yolo_srv)
        # response = gip_service()
        # camera.save_response(response)
        # if camera.is_updated and camera.found_objects:    
        #     camera.OblobsPublisher()

        self.callback_prediction(rospy.wait_for_message("/object_location", OBlobs))

        color = int(pnp.onion_color)
        if self.color != None:
            if self.color != color and self.color == 0:
                print '\nUpdating onion color as: ', self.color
            else:
                print '\nRetaining onion color as: ', color
                self.color = color
        else: pass

    def callback_prediction(self, msg):

        if max(msg.z) >= 0.85:
            idx = msg.x.index(max(msg.x))
            self.color = msg.color[idx]
            print '\nFound onion in hand. Color is: ', self.color
            # print '\nOnion z value: ', msg.z[idx]
        else:
            print "\nCouldn't find the onion in hand\n"
            # print 'Here are the zs: \n', msg.z


class Placeonconveyor(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                       input_keys=['counter'],
                       output_keys=['counter'])

    def execute(self, userdata):
        global pnp, current_state
        # rospy.loginfo('Executing state: Place')
        if userdata.counter >= 100:
            userdata.counter = 0
            return 'timed_out'

        place = pnp.placeOnConveyor()
        rospy.sleep(0.05)
        if place:
            detach = gripper_to_pos(0, 60, 200, False)    # GRIPPER TO POSITION 0
            lift = pnp.liftgripper()
            rospy.sleep(0.01)
            if lift:
                userdata.counter = 0
                current_state = vals2sid(ol=0, eefl=0, pred=2, listst=2)
                print '\nCurrent state after placing on conveyor: ', current_state
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'
        else:
            userdata.counter += 1
            return 'failed'


class Placeinbin(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                       input_keys=['counter'],
                       output_keys=['counter'])

    def execute(self, userdata):
        global pnp, current_state
        # rospy.loginfo('Executing state: Place')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        place = pnp.goto_bin()
        rospy.sleep(0.01)
        if place:
            userdata.counter = 0
            current_state = vals2sid(ol=2, eefl=2, pred=2, listst=2)
            print '\nCurrent state after placing in bin: ', current_state
            return 'success'
        else:
            userdata.counter += 1
            return 'failed'


class Detach_object(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                       input_keys=['counter'],
                       output_keys=['counter'])

    def execute(self, userdata):
        global pnp
        # rospy.loginfo('Executing state: Place')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'
        else:
            detach = gripper_to_pos(0, 60, 200, False)    # GRIPPER TO POSITION 0
            if detach:
                userdata.counter = 0
                rospy.sleep(2)  # Sleeping here because yolo catches the onion near bin while detaching and that screws up coordinates.
                '''NOTE: Both place on conveyor and bin use this, so don't update current state here.'''
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'


########################################################################################################################################

''' NOTE: The main function below is outdated now because many changes have been made in the
        states. You may have to update the code below if you need to test the state machine '''

# def main():
#     if len(sys.argv) < 2:
#         sortmethod = "pick"   # Default sort method
#     else:
#         sortmethod = sys.argv[1]
#
#     print("I'm in main!")
#
#     # Create a SMACH state machine
#     sm = StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED'])
#     sm.userdata.sm_x = []
#     sm.userdata.sm_y = []
#     sm.userdata.sm_z = []
#     sm.userdata.sm_color = []
#     sm.userdata.sm_counter = 0
#     sm.userdata.state = None
#     try:
#
#         # Open the container
#         with sm:
#             # Add states to the container
#             StateMachine.add('GETINFO', Get_info(),
#                             transitions={'updated':'CLAIM',
#                                         'not_updated':'GETINFO',
#                                         'timed_out': 'TIMED_OUT'},
#                             remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
#                                 'color':'sm_color','counter':'sm_counter'})
#             StateMachine.add('CLAIM', Claim(),
#                             transitions={'updated':'APPROACH',
#                                         'not_updated':'CLAIM',
#                                         'timed_out': 'TIMED_OUT',
#                                         'not_found': 'GETINFO',
#                                         'completed': 'SUCCEEDED'},
#                             remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
#                                 'color':'sm_color','counter':'sm_counter'})
#             StateMachine.add('APPROACH', Approach(),
#                             transitions={'success':'PICK',
#                                         'failed':'APPROACH',
#                                         'timed_out': 'TIMED_OUT'},
#                             remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
#                                 'color':'sm_color','counter':'sm_counter'})
#             StateMachine.add('PICK', Dipdown(),
#                             transitions={'success':'GRASP',
#                                         'failed':'PICK',
#                                         'timed_out': 'TIMED_OUT'},
#                             remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
#                                 'color':'sm_color','counter':'sm_counter'})
#
#             StateMachine.add('GRASP', Grasp_object(),
#                         transitions={'success':'LIFTUP',
#                                     'failed':'GRASP',
#                                     'timed_out': 'TIMED_OUT'},
#                         remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
#                                 'color':'sm_color','counter':'sm_counter'})
#
#             StateMachine.add('LIFTUP', Liftup(),
#                         transitions={'success':'VIEW',
#                                     'failed':'LIFTUP',
#                                     'timed_out': 'TIMED_OUT'},
#                         remapping={'counter':'sm_counter'})
#
#             StateMachine.add('VIEW', View(),
#                             transitions={'success':'PLACE',
#                                         'failed':'VIEW',
#                                         'timed_out': 'TIMED_OUT'},
#                             remapping={'color': 'sm_color','counter':'sm_counter'})
#             StateMachine.add('PLACE', Place(),
#                             transitions={'success':'DETACH',
#                                         'failed':'PLACE',
#                                         'timed_out': 'TIMED_OUT'},
#                             remapping={'color': 'sm_color','counter':'sm_counter'})
#             StateMachine.add('DETACH', Detach_object(),
#                             transitions={'success':'SUCCEEDED',
#                                         'failed':'DETACH',
#                                         'timed_out': 'TIMED_OUT'},
#                             remapping={'counter':'sm_counter'})
#
#         # Execute SMACH plan
#         outcome = sm.execute()
#         rospy.spin()
#
#     except rospy.ROSInterruptException:
#         return
#     except KeyboardInterrupt:
#         return


# if __name__ == '__main__':
#     print("Calling Main!")
#     try:
#         mymain()
#     except rospy.ROSInterruptException:
#         print "Main function not found! WTH dude!"
