#!/usr/bin/env python3
import rospy
from PickandPlace import PickAndPlace
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Int8MultiArray
from operator import mod
import random
import sys
from gripper_to_position import reset_gripper, activate_gripper, gripper_to_pos
import numpy as np

pnp = PickAndPlace()


def dummy():
    
    try:

        pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)

        # reset_gripper()

        # activate_gripper()

    #   # 255 = closed, 0 = open
        # gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER
        print("Hello")
        # rospy.sleep(1.0)
        # gripper_to_pos(255, 255, 200, False)    # GRIPPER TO POSITION 50
        # group = pnp.group
        # current_pose = group.get_current_pose().pose
        # print("\n",group.get_current_pose().pose.position)
        # allow_replanning = True
        # planning_time = 10
        # # state = pnp.robot.get_current_state()
        # # group.set_start_state(state)
        # pos = group.get_current_pose().pose.position
        # status = pnp.go_to_pose_goal(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w, .102, 0.32, 1.9, allow_replanning, planning_time, thresh = 0.001)
        # rospy.sleep(0.1)
        # print("\n",group.get_current_pose().pose.position)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
	try:
  		dummy()
	except rospy.ROSInterruptException:
	    pass
