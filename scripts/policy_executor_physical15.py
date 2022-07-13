#!/usr/bin/env python3
from time import time
import pnp_physical_vision as ppv
import rospkg

# Global variables
multi_frame = False

def ClaimNewOnion():
    global multi_frame
    # Create a SMACH state machine
    ClaimNewOnion = ppv.StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED', 'SORT COMPLETE'])
    ClaimNewOnion.userdata.sm_x = []
    ClaimNewOnion.userdata.sm_y = []
    ClaimNewOnion.userdata.sm_z = []
    ClaimNewOnion.userdata.sm_color = []
    ClaimNewOnion.userdata.sm_counter = 0
    try:
        # Open the container
        with ClaimNewOnion:
            # Add states to the container
            if multi_frame:
                ppv.StateMachine.add('GETINFO', ppv.Get_info_w_check(frame_threshold=3), 
                            transitions={'updated':'CLAIM', 
                                        'not_updated':'GETINFO',
                                        'timed_out': 'TIMED_OUT',
                                        'completed': 'SORT COMPLETE'},
                            remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                                'color':'sm_color','counter':'sm_counter'})
            else:
                ppv.StateMachine.add('GETINFO', ppv.Get_info(), 
                                transitions={'updated':'CLAIM', 
                                            'not_updated':'GETINFO',
                                            'timed_out': 'TIMED_OUT',
                                            'completed': 'SORT COMPLETE'},
                                remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                                    'color':'sm_color','counter':'sm_counter'})
            ppv.StateMachine.add('CLAIM', ppv.Claim(), 
                        transitions={'updated':'SUCCEEDED', 
                                    'not_updated':'CLAIM',
                                    'timed_out': 'TIMED_OUT',
                                    'not_found': 'GETINFO',
                                    'move': 'GETINFO',
                                    'completed': 'SORT COMPLETE'},
                        remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                            'color':'sm_color','counter':'sm_counter'})
        # Execute SMACH plan
        outcome_claim = ClaimNewOnion.execute()
        # print '\nClaim outcome is: ', outcome_claim
        # ppv.rospy.sleep(100)
        return outcome_claim

    except ppv.rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def Pick():
    # Create a SMACH state machine
    Pick = ppv.StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED', 'FAILED'])
    Pick.userdata.sm_x = []
    Pick.userdata.sm_y = []
    Pick.userdata.sm_z = []
    Pick.userdata.sm_color = []
    Pick.userdata.sm_counter = 0
    try:
        # Open the container
        with Pick:
            ppv.StateMachine.add('APPROACH', ppv.Approach(), 
                            transitions={'success':'CHECKNPICK', 
                                        'failed':'APPROACH',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                                'color':'sm_color','counter':'sm_counter'})
            ppv.StateMachine.add('CHECKNPICK', ppv.CheckNPick(), 
                            transitions={'success':'PICK', 
                                        'failed':'APPROACH',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                                'color':'sm_color','counter':'sm_counter'})
            ppv.StateMachine.add('PICK', ppv.Dipdown(), 
                            transitions={'success':'GRASP', 
                                        'failed':'PICK',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                                'color':'sm_color','counter':'sm_counter'})
            
            ppv.StateMachine.add('GRASP', ppv.Grasp_object(),
                        transitions={'success':'LIFTUP', 
                                    'failed':'GRASP',
                                    'timed_out': 'TIMED_OUT'},
                        remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                            'color':'sm_color','counter':'sm_counter'})
            ppv.StateMachine.add('LIFTUP', ppv.Liftup(),
                        transitions={'success': 'SUCCEEDED', 
                                    'failed':'LIFTUP',
                                    'no_grasp':'FAILED',
                                    'timed_out': 'TIMED_OUT'},
                        remapping={'counter':'sm_counter'})

        # Execute SMACH plan
        outcome_pick = Pick.execute()
        return outcome_pick

    except ppv.rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def InspectAfterPicking():
    # Create a SMACH state machine
    InspectAfterPicking = ppv.StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED'])
    InspectAfterPicking.userdata.sm_x = []
    InspectAfterPicking.userdata.sm_y = []
    InspectAfterPicking.userdata.sm_z = []
    InspectAfterPicking.userdata.sm_color = []
    InspectAfterPicking.userdata.sm_counter = 0
    try:
        # Open the container
        with InspectAfterPicking:
            ppv.StateMachine.add('VIEW', ppv.View(), 
                            transitions={'success':'SUCCEEDED', 
                                        'failed':'VIEW',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'color':'sm_color', 'counter':'sm_counter'})

        # Execute SMACH plan
        outcome_inspect = InspectAfterPicking.execute()
        return outcome_inspect

    except ppv.rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def PlaceInBin():
    # Create a SMACH state machine
    PlaceInBin = ppv.StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED'])
    PlaceInBin.userdata.sm_x = []
    PlaceInBin.userdata.sm_y = []
    PlaceInBin.userdata.sm_z = []
    PlaceInBin.userdata.sm_color = []
    PlaceInBin.userdata.sm_counter = 0
    try:
        # Open the container
        with PlaceInBin:
            ppv.StateMachine.add('PLACEINBIN', ppv.Placeinbin(), 
                            transitions={'success':'DETACH', 
                                        'failed':'PLACEINBIN',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'color': 'sm_color','counter':'sm_counter'})
            ppv.StateMachine.add('DETACH', ppv.Detach_object(),
                            transitions={'success':'SUCCEEDED', 
                                        'failed':'DETACH',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'counter':'sm_counter'})
        # Execute SMACH plan
        outcome_place = PlaceInBin.execute()
        return outcome_place

    except ppv.rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def PlaceOnConveyor():
    # Create a SMACH state machine
    PlaceOnConveyor = ppv.StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED'])
    PlaceOnConveyor.userdata.sm_x = []
    PlaceOnConveyor.userdata.sm_y = []
    PlaceOnConveyor.userdata.sm_z = []
    PlaceOnConveyor.userdata.sm_color = []
    PlaceOnConveyor.userdata.sm_counter = 0
    try:
        # Open the container
        with PlaceOnConveyor:    
            ppv.StateMachine.add('PLACEONCONVEYOR', ppv.Placeonconveyor(), 
                            transitions={'success':'SUCCEEDED', 
                                        'failed':'PLACEONCONVEYOR',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'color': 'sm_color','counter':'sm_counter'})
            ppv.StateMachine.add('DETACH', ppv.Detach_object(),
                            transitions={'success':'SUCCEEDED', 
                                        'failed':'DETACH',
                                        'timed_out': 'TIMED_OUT'},
                            remapping={'counter':'sm_counter'})
            # ppv.StateMachine.add('LIFTUP', ppv.Liftup(),
            #             transitions={'success': 'SUCCEEDED', 
            #                         'failed':'LIFTUP',
            #                         'timed_out': 'TIMED_OUT'},
            #             remapping={'counter':'sm_counter'})
        # Execute SMACH plan
        outcome_place = PlaceOnConveyor.execute()
        return outcome_place

    except ppv.rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def main():
    t0 = time()
    global multi_frame
    rospack = rospkg.RosPack()  # get an instance of RosPack with the default search paths
    path = rospack.get_path('ur3e_irl_project')   # get the file path for ur3e_irl_project

    import argparse

    desc = "A ROS Node to load a policy file and run a state machine based on policy actions. Use --help for usage instructions."
    # create parser
    parser = argparse.ArgumentParser(description = desc)

    # add arguments to the parser
    parser.add_argument('--policyfile', dest='policyfile', default='expert_policy.csv',
                help='Specify name of policy file (with extension) to load')
    parser.add_argument('--use_var_height', dest='var_height', action='store_true',
                help='Setting this True allows the usage of variable height values coming from the camera transforms') 
    parser.add_argument('--use_multiple_frames', dest='multi_frame', action='store_true',
                help='Setting this True allows the usage of multiple frames to average out the camera detections') 

    # parse the arguments
    args = parser.parse_args()
    print ("\n{} chosen with variable height: {} and multi_frame: {}".format(args.policyfile, args.var_height, args.multi_frame))
    multi_frame = args.multi_frame

    
    policy = ppv.np.genfromtxt(path+'/scripts/'+ args.policyfile, delimiter=' ')
    actList = {0:InspectAfterPicking, 1:PlaceOnConveyor, 2:PlaceInBin, 3:Pick, 4:ClaimNewOnion} 

    actMeaning = {0:'InspectAfterPicking', 1:'PlaceOnConveyor', 2:'PlaceInBin', 3:'Pick', 4:'ClaimNewOnion'} 

    ONIONLOC = {
    0: 'Unknown',
    1: 'OnConveyor',
    2: 'InFront',
    3: 'AtHome'
    }
    EEFLOC = {
        0: 'InBin',
        1: 'OnConveyor',
        2: 'InFront',
        3: 'AtHome'
    }

    PREDICTIONS = {
        0: 'Unknown',
        1: 'Bad',
        2: 'Good'
    }

    # ACTION_MEANING = {
    #     0: 'Noop',
    #     1: 'Detect',
    #     2: 'Pick',
    #     3: 'Inspect',
    #     4: 'PlaceOnConveyor',
    #     5: 'PlaceinBin'
    # }
    t1 = time()
    print("Time for init: ",t1-t0)
    ppv.rospy.init_node('policy_exec_phys', anonymous=True, disable_signals=False)
    ppv.pnp.goto_home(tolerance=0.1, goal_tol=0.1, orientation_tol=0.1)
    ppv.var_height = args.var_height

    outcome = actList[4]()
    t2 = time()
    print("Time before while loop: ", t2-t1)
    while not ppv.rospy.is_shutdown()  and outcome != 'SORT COMPLETE':
        print ('\n OUTCOME: ', outcome)
        # ppv.rospy.sleep(10)
        try:
            [oloc, eefloc, pred, _] = ppv.sid2vals(ppv.current_state)
            print (f'\nCurrent state is:\nOloc: {ONIONLOC[oloc]}, Eefloc: {EEFLOC[eefloc]}, Pred: {PREDICTIONS[pred]}\n')
            print ('\nExecuting action: ',actMeaning[policy[ppv.current_state]])
            outcome = actList[policy[ppv.current_state]]()
            if outcome == 'TIMED_OUT' or outcome == 'FAILED':
                print("\nTimed out/Failed, so going back to claim again!")
                outcome = actList[4]()
        except ppv.rospy.ROSInterruptException:
            ppv.rospy.signal_shutdown("Shutting down node, ROS interrupt received!")
        except KeyboardInterrupt:
            ppv.rospy.signal_shutdown("Shutting down node, Keyboard interrupt received!")
    # ppv.rospy.spin()
        
if __name__ == '__main__':
    # print("\nCalling Main!")
    try:
        main()
    except ppv.rospy.ROSInterruptException:
        print ("Main function not found! WTH dude!")
