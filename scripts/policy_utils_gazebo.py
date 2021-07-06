import numpy as np
nOnionLoc = 4
nEEFLoc = 4
nPredict = 3
nlistIDStatus = 3
nS = nOnionLoc*nEEFLoc*nPredict*nlistIDStatus
nA = 7
policy = np.genfromtxt('/home/psuresh/catkin_ws/src/ur3e_irl_project/scripts/expert_policy.csv', delimiter=' ')

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
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)
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