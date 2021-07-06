#!/usr/bin/env python

from PickandPlace import PickAndPlace

import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int8MultiArray
from operator import mod
import random
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import sys
from os import system
import rospy
from time import sleep
from ur3e_irl_project.msg import OBlobs
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import numpy as np
import _thread
import threading
import message_filters
from smach import *
from smach_ros import *
from smach_msgs.msg import *
import copy

# Global initializations
pnp = PickAndPlace()
done_onions = []
flag = False
total_onions = 4
attach_srv = None
detach_srv = None

class Get_info(State):
    def __init__(self):
        State.__init__(self, outcomes=['updated', 'not_updated', 'timed_out'],
                    input_keys = ['x','y','z','color','counter'],
                    output_keys = ['x','y','z','color','counter'])
        self.x = []
        self.y = []
        self.z = []
        self.color = []
        # rospy.Subscriber("/object_location", OBlobs, self.callback_vision)
        self.is_updated = False
        self.callback_vision(rospy.wait_for_message("/object_location", OBlobs))

    def callback_vision(self, msg):
        # print '\nCallback vision\n'
        while None in [msg.x,msg.y,msg.z,msg.color]:
            rospy.sleep(0.1)
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.color = msg.color
        self.is_updated = True
    
    def execute(self, userdata):
        # rospy.loginfo('Executing state: Get_info')
        if userdata.counter >= 500:
            userdata.counter = 0
            return 'timed_out'

        if self.is_updated == True:
            userdata.x = self.x
            userdata.y = self.y
            userdata.z = self.z
            userdata.color = self.color
            userdata.counter = 0
            rospy.sleep(0.05)
            return 'updated'
        else:
            userdata.counter += 1
            return 'not_updated'

class Claim(State):
    def __init__(self):
        State.__init__(self, outcomes=['updated', 'not_updated', 'timed_out','not_found', 'completed'],
                    input_keys = ['x','y','z','color','counter'],
                    output_keys = ['x','y','z','color','counter'])
        self.is_updated = False

    def execute(self, userdata):        
        global pnp, total_onions, attach_srv, detach_srv, done_onions
        while len(userdata.x) == 0:
            rospy.sleep(0.1)
        # rospy.loginfo('Executing state: Claim')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'
        if len(userdata.color) == 0:
            return 'not_found'
        max_index = len(userdata.color)
        # attach and detach service
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        detach_srv.wait_for_service()
        pnp.req.model_name_1 = None
        pnp.req.link_name_1 = "base_link"
        pnp.req.model_name_2 = "ur3e"
        pnp.req.link_name_2 = "right_l6"
        pnp.target_location_x = userdata.x[pnp.onion_index]
        pnp.target_location_y = userdata.y[pnp.onion_index]
        pnp.target_location_z = userdata.z[pnp.onion_index]
        # pnp.bad_onions = [i for i in range(pnp.onion_index, max_index) if (oi.color[i] == 0)]
        # print("Bad onion indices are: ", pnp.bad_onions)
        
        for i in range(total_onions):
            if len(done_onions) == total_onions:
                return 'completed'
            if i in done_onions:
                pass
            else:
                onion_guess = "onion_" + str(i)

                rospy.wait_for_service('/gazebo/get_model_state')
                try:
                    model_coordinates = rospy.ServiceProxy(
                        '/gazebo/get_model_state', GetModelState)
                    # print 'Onion name guessed: ', onion_guess
                    onion_coordinates = model_coordinates(onion_guess, "").pose
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e

                if pnp.target_location_y - 0.05 <= onion_coordinates.position.y <= pnp.target_location_y + 0.05:
                    if pnp.target_location_x - 0.05 <= onion_coordinates.position.x <= pnp.target_location_x + 0.05:
                        pnp.req.model_name_1 = onion_guess
                        done_onions.append(i)
                        self.is_updated = True
                        break

        if self.is_updated == False:
            userdata.counter += 1
            return 'not_updated'
        else:
            print "Onion name set as: ", pnp.req.model_name_1
            userdata.counter = 0
            rospy.sleep(0.05)
            return 'updated'

class Approach(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out','not_found'],
                    input_keys = ['x','y','z','color','counter'],
                    output_keys = ['x','y','z','color','counter'])

    def execute(self, userdata): 
        global pnp, total_onions
        # rospy.loginfo('Executing state: Approach')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        if userdata.x[pnp.onion_index] != pnp.target_location_x or \
            userdata.y[pnp.onion_index] != pnp.target_location_y or \
                userdata.z[pnp.onion_index] != pnp.target_location_z:
            home = pnp.goto_home(tolerance=0.1, goal_tol=0.1, orientation_tol=0.1)
            rospy.sleep(0.05)
            return 'not_found'
        else:
            # s = getState(pnp.req.model_name_1, 2)   # Sending unknown prediction until viewed
            # a = 3   # Pick
            # policy[s] = a
            home = pnp.goto_home(tolerance=0.1, goal_tol=0.1, orientation_tol=0.1)
            rospy.sleep(0.05)
            if home:
                status = pnp.goAndPick()
                rospy.sleep(0.05)
                if status:
                    userdata.counter = 0
                    return 'success'
                else:
                    userdata.counter += 1
                    return 'failed'
            else:
                userdata.counter += 1
                return 'failed'
            # grasps = pnp.make_grasps()   # generate a list of grasps
            # result = False
            # n_attempts = 0
            # robot = pnp.robot
            # manipulator = pnp.group
            # repeat until will succeed
            # while result == False:
            #     result = pnp.group.pick(pnp.req.model_name_1, grasps)      
            #     n_attempts += 1
            #     print "Attempts: ", n_attempts
            #     rospy.sleep(0.2)

class Pick(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out','not_found'],
                    input_keys = ['x','y','z','color','counter'],
                    output_keys = ['x','y','z','color','counter'])

    def execute(self, userdata): 
        global pnp, total_onions, attach_srv, detach_srv
        # rospy.loginfo('Executing state: Pick')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        # print '\nPICK State: Userdata.x\n', userdata.x

        # rospy.sleep(50)

        if userdata.x[pnp.onion_index] != pnp.target_location_x or \
            userdata.y[pnp.onion_index] != pnp.target_location_y or \
                userdata.z[pnp.onion_index] != pnp.target_location_z:
            return 'not_found'
        else:
            dip = pnp.staticDip(z_pose = 0.08)
            rospy.sleep(0.05)
            if dip:
                userdata.counter = 0
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'

class Grasp_object(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out','not_found'],
                    input_keys = ['x','y','z','color','counter'],
                    output_keys = ['x','y','z','color','counter'])

    def execute(self, userdata): 
        global pnp, total_onions, attach_srv, detach_srv
        # rospy.loginfo('Executing state: Pick')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        # print userdata.x

        # rospy.sleep(50)

        if userdata.x[pnp.onion_index] != pnp.target_location_x or \
            userdata.y[pnp.onion_index] != pnp.target_location_y or \
                userdata.z[pnp.onion_index] != pnp.target_location_z:
            return 'not_found'
        else:           
            at = attach_srv.call(pnp.req)
            rospy.sleep(0.1)
            if at:
                userdata.counter = 0
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'

class Liftup(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                        input_keys = ['counter'],
                        output_keys = ['counter'])

    def execute(self, userdata): 
        global pnp, total_onions, attach_srv, detach_srv
        # rospy.loginfo('Executing state: Pick')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'
        else:
            lift = pnp.liftgripper()
            rospy.sleep(0.05)
            if lift:
                userdata.counter = 0
                return 'success'
            else:
                detach_srv.call(pnp.req)
                userdata.counter += 1
                return 'failed'

class View(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                        input_keys = ['counter'],
                        output_keys = ['counter'])

    def execute(self, userdata): 
        global pnp, total_onions, attach_srv, detach_srv
        # rospy.loginfo('Executing state: View')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        view = pnp.view(0.3)
        rospy.sleep(0.01)
        if view:
            rotate = pnp.rotategripper(0.3)
            rospy.sleep(0.01)
            if rotate:
                userdata.counter = 0
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'
        else:
            userdata.counter += 1
            return 'failed'

class Place(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                        input_keys = ['color', 'counter'],
                        output_keys = ['color', 'counter'])

    def execute(self, userdata): 
        global pnp
        # rospy.loginfo('Executing state: Place')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        color = int(userdata.color[pnp.onion_index])
        if color:
            # s = getState(pnp.req.model_name_1, color)
            # a = 1   # Place on conveyor
            # policy[s] = a
            place = pnp.placeOnConveyor()
        else:
            # s = getState(pnp.req.model_name_1, color)
            # a = 2   # Place in bin
            # policy[s] = a
            place = pnp.goto_bin()
        rospy.sleep(0.01)
        if place:
            userdata.counter = 0
            return 'success'
        else:
            userdata.counter += 1
            return 'failed'

class Detach_object(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                        input_keys = ['counter'],
                        output_keys = ['counter'])

    def execute(self, userdata): 
        global pnp, detach_srv
        # rospy.loginfo('Executing state: Place')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'
        else:
            detach = detach_srv.call(pnp.req)
            if detach:
                userdata.counter = 0
                # s = getState(pnp.req.model_name_1, 2)   # Sending unknown prediction until viewed again
                # a = 4   # Claim new onion
                # policy[s] = a
                if pnp.onion_index < total_onions - 1:
                    pnp.onion_index = pnp.onion_index + 1
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'

def main():
    # print("I'm in main!")
    if len(sys.argv) < 2:
        sortmethod = "pick"   # Default sort method
    else:
        sortmethod = sys.argv[1]
    # Create a SMACH state machine
    sm = StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED'])
    sm.userdata.sm_x = []
    sm.userdata.sm_y = []
    sm.userdata.sm_z = []
    sm.userdata.sm_color = []
    sm.userdata.sm_counter = 0
    try:
        
        # Open the container
        with sm:
            # Add states to the container
            StateMachine.add('GETINFO', Get_info(), 
                            transitions={'updated':'CLAIM', 
                                        'not_updated':'GETINFO',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                                'color':'sm_color','counter':'sm_counter'})
            StateMachine.add('CLAIM', Claim(), 
                            transitions={'updated':'APPROACH', 
                                        'not_updated':'CLAIM',
                                        'timed_out': 'TIMED_OUT',
                                        'not_found': 'GETINFO',
                                        'completed': 'SUCCEEDED'},
                            remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                                'color':'sm_color','counter':'sm_counter'})
            StateMachine.add('APPROACH', Approach(), 
                            transitions={'success':'PICK', 
                                        'failed':'APPROACH',
                                        'timed_out': 'TIMED_OUT',
                                        'not_found': 'GETINFO'},
                            remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                                'color':'sm_color','counter':'sm_counter'})
            StateMachine.add('PICK', Pick(), 
                            transitions={'success':'GRASP', 
                                        'failed':'PICK',
                                        'timed_out': 'TIMED_OUT',
                                        'not_found': 'GETINFO'},
                            remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                                'color':'sm_color','counter':'sm_counter'})
            
            StateMachine.add('GRASP', Grasp_object(),
                        transitions={'success':'LIFTUP', 
                                    'failed':'GRASP',
                                    'timed_out': 'TIMED_OUT',
                                    'not_found': 'TIMED_OUT'},
                        remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                            'color':'sm_color','counter':'sm_counter'})
            StateMachine.add('LIFTUP', Liftup(),
                        transitions={'success':'VIEW', 
                                    'failed':'LIFTUP',
                                    'timed_out': 'TIMED_OUT'},
                        remapping={'counter':'sm_counter'})

            StateMachine.add('VIEW', View(), 
                            transitions={'success':'PLACE', 
                                        'failed':'VIEW',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'counter':'sm_counter'})
            StateMachine.add('PLACE', Place(), 
                            transitions={'success':'DETACH', 
                                        'failed':'PLACE',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'color': 'sm_color','counter':'sm_counter'})
            StateMachine.add('DETACH', Detach_object(),
                        transitions={'success':'SUCCEEDED', 
                                    'failed':'DETACH',
                                    'timed_out': 'TIMED_OUT'})

        # Execute SMACH plan
        outcome = sm.execute()
        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    # print("Calling Main!")
    try:
        main()
    except rospy.ROSInterruptException:
        print "Main function not found! WTH dude!"