#!/usr/bin/env python3
import rospy
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from time import sleep
from std_msgs.msg import String
from std_srvs.srv import Trigger

def reset_gripper():
    return True

def activate_gripper():
    return True


def gripper_to_pos(pos, force, speed, hp):

#    rospy.loginfo("I'm in the function")

    min_open = 24.0
    max_open = 117.0
    
    position = int((max_open - min_open) * (float(255 - pos) / 255.0) + min_open)
    
    pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
    while pub.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect")
        rospy.sleep(0.1)
    
    command = """def gripperProgram():
  on_xmlrpc = rpc_factory("xmlrpc", "http://localhost:41414")
  retVal=on_xmlrpc.sg_grip(0,"""
  
    command += str(position) + ",True,False)\nend\n"


    pub.publish(command)

    rospy.sleep(1)    
    rate = rospy.Rate(10) # 10hz
    
    resend = rospy.ServiceProxy('/ur_hardware_interface/resend_robot_program', Trigger)

    resend()
    
    rate.sleep()

    return True
