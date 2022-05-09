#!/usr/bin/env python3

import sys
import rospkg

rospack = rospkg.RosPack()  # get an instance of RosPack with the default search paths
# get the file path for sanet_onionsorting
path = rospack.get_path('sanet_onionsorting')
sys.path.append(path + '/scripts/')

from rgbd_imgpoint_to_tf import Camera
from sanet_onionsorting.srv import yolo_srv
from ur3e_irl_project.msg import OBlobs
import rospy

rgbtopic = '/camera/color/image_raw'
depthtopic = '/camera/aligned_depth_to_color/image_raw'
camerainfo = '/camera/color/camera_info'
choice = 'real'
debug = False
init_node = False


def camera_node():
    print("We're in camera node function!")
    camera = Camera(rgbtopic, depthtopic, camerainfo, choice, debug, init_node)
    rospy.wait_for_service("/get_predictions")  # Contains the centroids of the obj bounding boxes
    gip_service = rospy.ServiceProxy("/get_predictions", yolo_srv)
    response = gip_service()
    # print(f"Response: {response}")
    camera.save_response(response)
    print("\nIs updated: ",camera.is_updated,"\tFound objects: ", camera.found_objects)
    if camera.is_updated and camera.found_objects:    
        camera.OblobsPublisher()
    return