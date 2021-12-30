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


def main():
    
    try:

        group = pnp.group
        allow_replanning = True
        planning_time = 10
        pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
        rospy.sleep(0.01)
        # pnp.goto_placeOnConv()
        # pnp.goto_bin(usePoseGoal=False)
        # reset_gripper()

        # activate_gripper()

    #   # 255 = closed, 0 = open
        # rospy.sleep(0.5)
        # gripper_to_pos(60, 60, 200, False)    # OPEN GRIPPER
        # # gripper_to_pos(255, 255, 200, False)    # GRIPPER TO POSITION 255
        current_pose = group.get_current_pose().pose
        # print("Current pose is: ")
        # print("\n", current_pose)
        # # # state = pnp.robot.get_current_state()
        # # # group.set_start_state(state)
        status = pnp.go_to_pose_goal(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w, 0, 0.45, 0.95, allow_replanning, planning_time, thresh = 0.001)
        # rospy.sleep(0.01)
        # gr = gripper_to_pos(150, 60, 200, False)    # GRIPPER TO POSITION 150
        # rospy.sleep(0.05)
        pnp.target_location_x = 10
        lift = pnp.liftgripper()
        # # pos = group.get_current_pose().pose.position
        # # pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
        # rospy.sleep(0.01)
        # view = pnp.view()     # We're rotating to show the camera behind UR
        # pnp.goto_bin()
        # pnp.placeOnConveyor()
        print("After going to pose goal:\n",group.get_current_pose().pose.position)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
	try:
  		main()
	except rospy.ROSInterruptException:
	    pass
