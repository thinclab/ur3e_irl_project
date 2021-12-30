#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Int8MultiArray
from moveit_commander.conversions import pose_to_list
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from tf.transformations import quaternion_from_euler
from ur3e_irl_project.msg import onions_blocks_poses
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
import copy
import random


class PickAndPlace(object):
    def __init__(self, init_node = True, limb='right', tip_name="right_gripper_tip"):
        #super(PickAndPlace, self).__init__()

        # print("Class init happening bro!")
        joint_state_topic = ['joint_states:=/joint_states']
        # at HOME position, orientation of gripper frame w.r.t world x=0.7, y=0.7, z=0.0, w=0.0 or [ rollx: -3.1415927, pitchy: 0, yawz: -1.5707963 ]
        # (roll about an X-axis w.r.t home) / (subsequent pitch about the Y-axis) / (subsequent yaw about the Z-axis)
        rollx = -3.1415926535
        pitchy = 0.0
        yawz = 0.0    # -1.5707963
        # use q with moveit because giving quaternion.x doesn't work.
        q = quaternion_from_euler(rollx, pitchy, yawz)
        q[0] = 1
        q[1] = 0
        q[2] = 0
        q[3] = 0
   
        overhead_orientation_moveit = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3])

        moveit_commander.roscpp_initialize(joint_state_topic)

        # moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"

        group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=5.0)
        # See ompl_planning.yaml for a complete list
        group.set_planner_id("RRTConnectkConfigDefault")
        # group.set_planner_id("BiEST")
        # group.set_planner_id("trajopt_interface/TrajOptPlanner")
        # group.set_planner_id("LazyPRMStar")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = group.get_planning_frame()

        eef_link = group.get_end_effector_link()

        group_names = robot.get_group_names()

        req = AttachRequest()
        if init_node:
            print("Initializing rosnode - pnp_node")
            rospy.init_node('pnp_node', anonymous=True, disable_signals=False)
            self._limb_name = limb  # string
#            self._limb = intera_interface.Limb(limb)
#            self._tip_name = tip_name
        # print(robot.get_current_state())

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.group = group
        self.req = req
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.q = q
        self.overhead_orientation_moveit = overhead_orientation_moveit
        self.target_location_x = -100
        self.target_location_y = -100
        self.target_location_z = -100
        self.onion_index = 0
        self.onion_color = None
        # self.num_onions = 0
        # self.bad_onions = []
        # self.onionLoc = None
        # self.eefLoc = None
        # self.prediction = None
        # self.listIDstatus = None

    def all_close(self, goal, actual, tolerance = 0.01):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal: A list of floats, a Pose or a PoseStamped
        @param: actual: list of floats, a Pose or a PoseStamped
        @param: tolerance: A float
        """

        if type(goal) is list:
            for index in [0, 1, 2]:
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            # print "pose_to_list(goal):"+str(pose_to_list(goal))
            # print "pose_to_list(actual):"+str(pose_to_list(actual))
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

    def move_to_start(self, start_angles=None):
        # print("Moving the {0} arm to pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles, timeout=2.0)
        return True

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            #move_to_joint_positions(self, positions, timeout=15.0,threshold=settings.JOINT_ANGLE_TOLERANCE,test=None)
            self._limb.move_to_joint_positions(joint_angles, timeout=timeout)
        else:
            rospy.logerr(
                "No Joint Angles provided for move_to_joint_positions. Staying put.")

    # This is an intera based IK method - Doesn't know about collision object. This is currently not used!
    def _servo_to_pose(self, pose, time=4.0, steps=400.0):
        ''' An *incredibly simple* linearly-interpolated Cartesian move '''
        r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
        current_pose = self._limb.endpoint_pose()
        print("Current pose is: \n",current_pose)
        ik_delta = Pose()
        ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps
        ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
        ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
        ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
        ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
        ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
        ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps
        # print("ik_delta z pose: ",ik_delta.position.z)
        for d in range(int(steps), -1, -1):
            if rospy.is_shutdown():
                return
            ik_step = Pose()
            ik_step.position.x = d*ik_delta.position.x + pose.position.x
            ik_step.position.y = d*ik_delta.position.y + pose.position.y
            ik_step.position.z = d*ik_delta.position.z + pose.position.z
            ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
            ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
            ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
            ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
            joint_angles = self._limb.ik_request(ik_step, self._tip_name)
            # print("ik_step z pose: ",ik_step.position.z)
            print("These are the joint angles I got: ",joint_angles)
            # rospy.sleep(500)
            if joint_angles:
                self._limb.set_joint_positions(joint_angles)
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            r.sleep()
        return True

    def go_to_joint_goal(self, angles, allow_replanning=True, planning_time=10.0,
                         goal_tol=0.01, orientation_tol=0.01):

        group = self.group
        # Allow some leeway in position(meters) and orientation (radians)
        group.set_goal_position_tolerance(goal_tol)
        group.set_goal_orientation_tolerance(goal_tol)
        group.allow_replanning(allow_replanning)
        group.set_planning_time(planning_time)
        group.go(angles, wait=True)
        group.stop()
        return True

    def go_to_pose_goal(self, ox, oy, oz, ow, px, py, pz, allow_replanning=True, planning_time=10.0, thresh = 0.025):

        group = self.group
        # Allow some leeway in position(meters) and orientation (radians)
        group.set_goal_position_tolerance(thresh)
        group.set_goal_orientation_tolerance(thresh)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = ox
        pose_goal.orientation.y = oy
        pose_goal.orientation.z = oz
        pose_goal.orientation.w = ow
        pose_goal.position.x = px
        pose_goal.position.y = py
        pose_goal.position.z = pz
        group.set_pose_target(pose_goal)
        group.allow_replanning(allow_replanning)
        group.set_planning_time(planning_time)
        plan = group.go(wait=True)
        # rospy.sleep(1)
        group.stop()

        group.clear_pose_targets()

        # current_pose = group.get_current_pose().pose
        # return self.all_close(pose_goal, current_pose, thresh)
        return True

    def wait_for_state_update(self, box_name, scene, box_is_known=False, box_is_attached=False, timeout=5):
        """ This func is used when we need to add onion collision to moveit """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.05)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    # Get the gripper posture as a JointTrajectory

    def make_gripper_posture(self, pose):
        t = JointTrajectory()
        # t.joint_names = ['finger_joint','left_inner_finger_joint','left_inner_knuckle_joint','left_inner_knuckle_dummy_joint','right_inner_knuckle_joint','right_inner_knuckle_dummy_joint','right_outer_knuckle_joint','right_inner_finger_joint']
        t.joint_names = ['finger_joint']
        tp = JointTrajectoryPoint()
        tp.positions = [pose for j in t.joint_names]
        tp.effort = [0.0]
        t.points.append(tp)
        return t

    def make_gripper_translation(self, min_dist, desired, axis=-1.0):
        g = GripperTranslation()
        g.direction.vector.z = axis
        g.direction.header.frame_id = 'right_l6'
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    def make_grasps(self):
        # setup defaults for the grasp
        g = Grasp()
        # These two lines are for gripper open and close.
        g.pre_grasp_posture = self.make_gripper_posture(0)
        # They don't matter in current simulation.
        g.grasp_posture = self.make_gripper_posture(0)
        g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.1)
        g.post_grasp_retreat = self.make_gripper_translation(0.05, 0.1, 1.0)
        g.grasp_pose = self.group.get_current_pose()

        # generate list of grasps
        grasps = []
        g.grasp_pose.pose.orientation.x = self.q[0]
        g.grasp_pose.pose.orientation.y = self.q[1]
        g.grasp_pose.pose.orientation.z = self.q[2]
        g.grasp_pose.pose.orientation.w = self.q[3]
        g.id = str(len(grasps))
        g.allowed_touch_objects = [self.req.model_name_1]
        g.max_contact_force = 0
        grasps.append(copy.deepcopy(g))
        print('Successfully created grasps!')
        return grasps


#####################################################################################################################################


    def dip(self):
        group = self.group
        while self.target_location_x == -100:
            rospy.sleep(0.05)
        current_pose = group.get_current_pose().pose

        allow_replanning = True
        planning_time = 10
        threshold = 0.05
        if current_pose.position.y > self.target_location_y and current_pose.position.y < self.target_location_y + 0.005:
            # print "Now performing dip"
            dip = self.go_to_pose_goal(self.q[0], self.q[1], self.q[2], self.q[3], self.target_location_x - 0.025,  # Account for tol
                                       self.target_location_y + 0.02,  # accounting for tolerance error
                                       current_pose.position.z - 0.075,  # This is where we dip
                                       allow_replanning, planning_time,threshold)
            rospy.sleep(0.05)

            # print "Successfully dipped! z pos: ", current_pose.position.z
            return True
        else:
            rospy.sleep(0.05)
            # print "Current position of gripper (y,z): ", current_pose.position.y, current_pose.position.z
            # print "Current position of onion in (y): ", self.target_location_y
            dip = self.dip()
        return dip

    def waitToPick(self):

        group = self.group
        while self.target_location_x == -100:
            rospy.sleep(0.05)
        current_pose = group.get_current_pose().pose
        allow_replanning = True
        planning_time = 10
        # print "Attempting to reach {},{},{}".format(self.target_location_x,
        #                                             self.target_location_y,
        #                                             current_pose.position.z)
        threshold = 0.05
        waiting = self.go_to_pose_goal(self.q[0], self.q[1], self.q[2], self.q[3], self.target_location_x,
                                       self.target_location_y + 0.1,  # Going to hover location .1 from the onion
                                       current_pose.position.z,
                                       allow_replanning, planning_time, threshold)
        rospy.sleep(0.05)
        dip = self.dip()
        return dip

    def goAndPick(self):

        group = self.group
        while self.target_location_x == -100:
            rospy.sleep(0.05)
        current_pose = group.get_current_pose().pose
        allow_replanning = False
        planning_time = 10
        print("\nAttempting to reach {},{},{}".format(self.target_location_x,
                                                     self.target_location_y,
                                                     current_pose.position.z))
        threshold = 0.02
        status = self.go_to_pose_goal(self.q[0], self.q[1], self.q[2], self.q[3], self.target_location_x,
                                       self.target_location_y,
                                        self.target_location_z+.225,      # - 0.045,1.2114413504613049
                                       allow_replanning, planning_time, threshold)
        rospy.sleep(0.05)
        # dip = self.staticDip()
        # return dip
        return status

    def staticDip(self, gripper_length = 0.1, tolerance=0.055):
        
        group = self.group
        current_pose = group.get_current_pose().pose
        position_constraint = PositionConstraint()
        position_constraint.target_point_offset.x = 0.1
        position_constraint.target_point_offset.y = 0.1
        position_constraint.target_point_offset.z = 0.5
        position_constraint.weight = 0.25
        position_constraint.link_name = group.get_end_effector_link()
        position_constraint.header.frame_id = group.get_planning_frame()
        orientation_constraint = OrientationConstraint()
        orientation_constraint.orientation = group.get_current_pose().pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.3
        orientation_constraint.absolute_y_axis_tolerance = 0.3
        orientation_constraint.absolute_z_axis_tolerance = 0.3
        orientation_constraint.weight = 0.5
        orientation_constraint.link_name = group.get_end_effector_link()
        orientation_constraint.header.frame_id = group.get_planning_frame()

        constraint = Constraints()
        constraint.orientation_constraints.append(orientation_constraint)
        constraint.position_constraints.append(position_constraint)
        group.set_path_constraints(constraint)
        allow_replanning = False
        planning_time = 10
        before_dip = current_pose.position.z
        # dip = False
        # while not dip: 
        # print(self.target_location_x, self.target_location_y, gripper_length)
        dip = self.go_to_pose_goal(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w, self.target_location_x,  # accounting for tolerance error
                                self.target_location_y,  # accounting for tolerance error
                                self.target_location_z + gripper_length,  # For UR, eef frame is wrist_3, so we add the gripper_length to that for right amount of dip
                                allow_replanning, planning_time, tolerance/3)
        current_pose = group.get_current_pose().pose
        print("after dip", current_pose)
        rospy.sleep(0.01)
        group.clear_path_constraints()
        after_dip = group.get_current_pose().pose.position.z
        if dip and (before_dip > after_dip):
            # print "\nBefore dip z pose: ",before_dip
            # print "\nAfter dip z pose: ",after_dip
            # print "\nSuccessfully dipped!"
            return True
        else:
            self.staticDip()
        # return True


    def liftgripper(self, threshold = 0.025):
        # approx centers of onions at 0.82, width of onion is 0.038 m. table is at 0.78
        # length of gripper is 0.163 m The gripper should not go lower than
        # (height_z of table w.r.t base+gripper-height/2+tolerance) = 0.78-0.93+0.08+0.01=-0.24
        # pnp._limb.endpoint_pose returns {'position': (x, y, z), 'orientation': (x, y, z, w)}
        # moving from z=-.02 to z=-0.1

        print("Attempting to lift gripper")
        group = self.group
        while self.target_location_x == -100:
            rospy.sleep(0.05)
        position_constraint = PositionConstraint()
        position_constraint.target_point_offset.x = 0.1
        position_constraint.target_point_offset.y = 0.1
        position_constraint.target_point_offset.z = 0.5
        position_constraint.weight = 0.25
        position_constraint.link_name = group.get_end_effector_link()
        position_constraint.header.frame_id = group.get_planning_frame()
        orientation_constraint = OrientationConstraint()
        orientation_constraint.orientation = group.get_current_pose().pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.3
        orientation_constraint.absolute_y_axis_tolerance = 0.3
        orientation_constraint.absolute_z_axis_tolerance = 0.3
        orientation_constraint.weight = 0.5   # Empirically estimated values for ur3e Robot
        orientation_constraint.link_name = group.get_end_effector_link()
        orientation_constraint.header.frame_id = group.get_planning_frame()

        constraint = Constraints()
        constraint.orientation_constraints.append(orientation_constraint)
        constraint.position_constraints.append(position_constraint)
        group.set_path_constraints(constraint)
        allow_replanning = False
        planning_time = 10
        lifted = False
        current_pose = group.get_current_pose().pose
        print("Current z pose: ", current_pose.position.z)
        z_pose = current_pose.position.z + 0.125
        while not lifted:
            lifted = self.go_to_pose_goal(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w, current_pose.position.x,
                                           current_pose.position.y, 
                                           z_pose,
                                           allow_replanning, planning_time, threshold)
            # current_pose = group.get_current_pose().pose
            rospy.sleep(0.01)
        group.clear_path_constraints()
        # print("Successfully lifted gripper to z: ", current_pose.position.z)

        return True
###################################################################################################################################

    def display_trajectory(self, plan):
        """
        Display a movement plan / trajectory
        @param: plan: Plan to be displayed
        """
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)

    # This method uses intera to move because we assume there are no
    # obstacles on the way to the start pose.
    def goto_home(self, tolerance=0.001, goal_tol=0.001, orientation_tol=0.001):

        # print("Attempting to reach home\n")
        group = self.group
        home_joint_angles = [-1.2912285963641565, -1.3949199479869385, -1.3669726848602295, -1.8911224804320277, 1.6427865028381348, -0.0064538160907190445]
        # home_joint_angles = [-1.3669726848602295, -1.3949199479869385, -1.2912285963641565, -1.8911224804320277, 1.6427865028381348, -0.0064538160907190445]

        joint_angles = {'elbow_joint': home_joint_angles[0],
                        'shoulder_lift_joint': home_joint_angles[1],
                        'shoulder_pan_joint': home_joint_angles[2],
                        'wrist_1_joint': home_joint_angles[3],
                        'wrist_2_joint': home_joint_angles[4],
                        'wrist_3_joint': home_joint_angles[5]}
        # 0.7716502133436203, -0.25253308083711357, -0.9156571119870254, 1.6775039734444164, 2.969104448028304, -2.2600790124759307, -2.608939978894689
        current_joints = self.group.get_current_joint_values()
        tol = tolerance
        diff = abs(joint_angles['elbow_joint']-current_joints[0]) > tol or \
            abs(joint_angles['shoulder_lift_joint']-current_joints[1]) > tol or \
            abs(joint_angles['shoulder_pan_joint']-current_joints[2]) > tol or \
            abs(joint_angles['wrist_1_joint']-current_joints[3]) > tol or \
            abs(joint_angles['wrist_2_joint']-current_joints[4]) > tol or \
            abs(joint_angles['wrist_3_joint']-current_joints[5]) > tol

        while diff:
            self.go_to_joint_goal(home_joint_angles, True, 5.0, goal_tol=goal_tol,
                                  orientation_tol=orientation_tol)
            rospy.sleep(0.05)
            # measure after movement
            current_joints = group.get_current_joint_values()

            diff = abs(joint_angles['elbow_joint']-current_joints[0]) > tol or \
                abs(joint_angles['shoulder_lift_joint']-current_joints[1]) > tol or \
                abs(joint_angles['shoulder_pan_joint']-current_joints[2]) > tol or \
                abs(joint_angles['wrist_1_joint']-current_joints[3]) > tol or \
                abs(joint_angles['wrist_2_joint']-current_joints[4]) > tol or \
                abs(joint_angles['wrist_3_joint']-current_joints[5]) > tol
            if diff:
#                self.move_to_start(joint_angles)
                rospy.sleep(0.05)

            # print("diff:"+str(diff))
        # self.go_to_joint_goal(joint_angles)
        # print("reached home")
        return True

    def view(self, tol=0.01, goal_tol=0.01, orientation_tol=0.01):

        print("Attempting to reach viewpoint\n")
        group = self.group
        view_joint_angles = [-1.2217128276824951, -2.1831122837462367, -1.3320048491107386, -3.136397501031393, 2.564589023590088, 0.04503623768687248]
        joint_angles = {'elbow_joint': view_joint_angles[0],
                        'shoulder_lift_joint': view_joint_angles[1],
                        'shoulder_pan_joint': view_joint_angles[2],
                        'wrist_1_joint': view_joint_angles[3],
                        'wrist_2_joint': view_joint_angles[4],
                        'wrist_3_joint': view_joint_angles[5]}
        current_joints = self.group.get_current_joint_values()
        diff = abs(joint_angles['elbow_joint']-current_joints[0]) > tol or \
            abs(joint_angles['shoulder_lift_joint']-current_joints[1]) > tol or \
            abs(joint_angles['shoulder_pan_joint']-current_joints[2]) > tol or \
            abs(joint_angles['wrist_1_joint']-current_joints[3]) > tol or \
            abs(joint_angles['wrist_2_joint']-current_joints[4]) > tol or \
            abs(joint_angles['wrist_3_joint']-current_joints[5]) > tol

        while diff:
            self.go_to_joint_goal(view_joint_angles, True, 5.0, goal_tol=goal_tol,
                                  orientation_tol=orientation_tol)
            rospy.sleep(0.05)
            # measure after movement
            current_joints = group.get_current_joint_values()

            diff = abs(joint_angles['elbow_joint']-current_joints[0]) > tol or \
                abs(joint_angles['shoulder_lift_joint']-current_joints[1]) > tol or \
                abs(joint_angles['shoulder_pan_joint']-current_joints[2]) > tol or \
                abs(joint_angles['wrist_1_joint']-current_joints[3]) > tol or \
                abs(joint_angles['wrist_2_joint']-current_joints[4]) > tol or \
                abs(joint_angles['wrist_3_joint']-current_joints[5]) > tol
            if diff:
#                self.move_to_start(joint_angles)
                rospy.sleep(0.05)

            # print("diff:"+str(diff))
        # self.go_to_joint_goal(joint_angles)
        # print("reached view")
        return True


    def rotategripper(self, tolerance=0.01, goal_tol=0.01, orientation_tol=0.01):

        group = self.group
        current_joints = group.get_current_joint_values()
        # print('\nCurrent Joints: \n',current_joints)

        if (current_joints[5] + 3.1415926535) >= 4.712389:  # If rotating in one direction exceeds max angle(270), go the other way.
            clockwise = {'elbow_joint': current_joints[0],
                        'shoulder_lift_joint': current_joints[1],
                        'shoulder_pan_joint': current_joints[2],
                        'wrist_1_joint': current_joints[3],
                        'wrist_2_joint': current_joints[4],
                        'wrist_3_joint': current_joints[5] - (3.1415926535 / 2)}
        else:
            clockwise = {'elbow_joint': current_joints[0],
                        'shoulder_lift_joint': current_joints[1],
                        'shoulder_pan_joint': current_joints[2],
                        'wrist_1_joint': current_joints[3],
                        'wrist_2_joint': current_joints[4],
                        'wrist_3_joint': current_joints[5] + (3.1415926535 / 2)}

#        done = self.go_to_joint_goal(clockwise)
        done = True     # NOTE: We have disabled rotation by bypassing the prev line, uncomment prev line and comment this to implement rotation.
        rospy.sleep(0.01)

        return done
    
    def goto_bin(self, tolerance=0.01, goal_tol=0.01, orientation_tol=0.01, usePoseGoal = False):

        group = self.group
        allow_replanning = False
        planning_time = 10
        reached = False
        # print "Attempting to reach the bin"
        if usePoseGoal:

            while not reached:
                reached = self.go_to_pose_goal(self.q[0], self.q[1], self.q[2], self.q[3], 0.5, -0.08, 1,
                                            allow_replanning, planning_time, tolerance)
                rospy.sleep(0.02)

            print("Reached bin: ", reached)
            # current_pose = group.get_current_pose().pose
            # print "current_pose: " + str((current_pose))

        else:

            group = self.group
            bin_joint_angles = [-2.8, -2.5122820339598597, -0.61946702003479, -1.5734607181944789, 1.5675392150878906, -1.4726555983172815]

            joint_angles = {'elbow_joint': bin_joint_angles[0],
                            'shoulder_lift_joint': bin_joint_angles[1],
                            'shoulder_pan_joint': bin_joint_angles[2],
                            'wrist_1_joint': bin_joint_angles[3],
                            'wrist_2_joint': bin_joint_angles[4],
                            'wrist_3_joint': bin_joint_angles[5]}
            # 0.7716502133436203, -0.25253308083711357, -0.9156571119870254, 1.6775039734444164, 2.969104448028304, -2.2600790124759307, -2.608939978894689
            current_joints = self.group.get_current_joint_values()
            tol = tolerance
            diff = abs(joint_angles['elbow_joint']-current_joints[0]) > tol or \
                abs(joint_angles['shoulder_lift_joint']-current_joints[1]) > tol or \
                abs(joint_angles['shoulder_pan_joint']-current_joints[2]) > tol or \
                abs(joint_angles['wrist_1_joint']-current_joints[3]) > tol or \
                abs(joint_angles['wrist_2_joint']-current_joints[4]) > tol or \
                abs(joint_angles['wrist_3_joint']-current_joints[5]) > tol

            while diff:
                self.go_to_joint_goal(bin_joint_angles, allow_replanning, planning_time, goal_tol=goal_tol,
                                    orientation_tol=orientation_tol)
                rospy.sleep(0.05)
                # measure after movement
                current_joints = group.get_current_joint_values()

                diff = abs(joint_angles['elbow_joint']-current_joints[0]) > tol or \
                    abs(joint_angles['shoulder_lift_joint']-current_joints[1]) > tol or \
                    abs(joint_angles['shoulder_pan_joint']-current_joints[2]) > tol or \
                    abs(joint_angles['wrist_1_joint']-current_joints[3]) > tol or \
                    abs(joint_angles['wrist_2_joint']-current_joints[4]) > tol or \
                    abs(joint_angles['wrist_3_joint']-current_joints[5]) > tol
                if diff:
    #                self.move_to_start(joint_angles)
                    rospy.sleep(0.05)
            reached = True       
        return reached

    def goto_placeOnConv(self, tolerance=0.01, goal_tol=0.01, orientation_tol=0.01):

        allow_replanning = False
        planning_time = 10
        print("Attempting to reach place on conv\n")
        group = self.group
        conv_joint_angles = [-0.2363308221101761, -2.3734213314452113, -0.4973037878619593, -2.0139085255064906, 1.5427601337432861, 0.05646385997533798]

        joint_angles = {'elbow_joint': conv_joint_angles[0],
                            'shoulder_lift_joint': conv_joint_angles[1],
                            'shoulder_pan_joint': conv_joint_angles[2],
                            'wrist_1_joint': conv_joint_angles[3],
                            'wrist_2_joint': conv_joint_angles[4],
                            'wrist_3_joint': conv_joint_angles[5]}
            # 0.7716502133436203, -0.25253308083711357, -0.9156571119870254, 1.6775039734444164, 2.969104448028304, -2.2600790124759307, -2.608939978894689
        current_joints = self.group.get_current_joint_values()
        tol = tolerance
        diff = abs(joint_angles['elbow_joint']-current_joints[0]) > tol or \
            abs(joint_angles['shoulder_lift_joint']-current_joints[1]) > tol or \
            abs(joint_angles['shoulder_pan_joint']-current_joints[2]) > tol or \
            abs(joint_angles['wrist_1_joint']-current_joints[3]) > tol or \
            abs(joint_angles['wrist_2_joint']-current_joints[4]) > tol or \
            abs(joint_angles['wrist_3_joint']-current_joints[5]) > tol

        while diff:
            self.go_to_joint_goal(conv_joint_angles, allow_replanning, planning_time, goal_tol=goal_tol,
                                orientation_tol=orientation_tol)
            rospy.sleep(0.05)
            # measure after movement
            current_joints = group.get_current_joint_values()

            diff = abs(joint_angles['elbow_joint']-current_joints[0]) > tol or \
                abs(joint_angles['shoulder_lift_joint']-current_joints[1]) > tol or \
                abs(joint_angles['shoulder_pan_joint']-current_joints[2]) > tol or \
                abs(joint_angles['wrist_1_joint']-current_joints[3]) > tol or \
                abs(joint_angles['wrist_2_joint']-current_joints[4]) > tol or \
                abs(joint_angles['wrist_3_joint']-current_joints[5]) > tol
            if diff:
#                self.move_to_start(joint_angles)
                rospy.sleep(0.05)
        return True


    def placeOnConveyor(self, tolerance=0.05, goal_tol=0.01, orientation_tol=0.01):

        onConveyor = False
        allow_replanning = False
        planning_time = 5
        group = self.group
        ori = group.get_current_pose().pose.orientation
        miny = 0.25
        maxy = 0.45
        minx = -0.45
        maxx = -0.25
        yval = miny + (maxy-miny)*random.random()
        xval = minx + (maxx-minx)*random.random()
        current_pose = self.group.get_current_pose().pose
        onConveyor = self.go_to_pose_goal(ori.x,ori.y,ori.z,ori.w, xval, yval, current_pose.position.z,
                                            allow_replanning, planning_time, tolerance)
        rospy.sleep(0.02)
        # current_pose = group.get_current_pose().pose
        # print("Current gripper pose: ", current_pose)
        print("Over the conveyor now!")
        return onConveyor

############################################## END OF CLASS ##################################################
