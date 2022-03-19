#!/usr/bin/env python3

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

def main():
    # print("I'm in main!")
    try:
        group = pnp.group
        allow_replanning = True
        planning_time = 10
        pnp.goto_home()
        pnp.target_location_x = 0
        pnp.target_location_y = 0.4
        pnp.target_location_z = 0.78
        current_pose = group.get_current_pose().pose
        # status = pnp.go_to_pose_goal(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w, 0, 0.45, 0.95, allow_replanning, planning_time, thresh = 0.001)
        rospy.sleep(0.1)
        pnp.staticDip(gripper_length=0.162)
        # print "\n",group.get_current_pose().pose.position
        rospy.spin()


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    print("Calling Main!")
    try:
        main()
    except rospy.ROSInterruptException:
        print("Main function not found! WTH dude!")
