#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from moveit_commander.conversions import pose_to_list
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
import random
import shape_msgs.msg
import sensor_msgs.msg
import visualization_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler, quaternion_multiply
from six.moves import input  # Python 3 compatible alternative for raw_input

# define some colours for convenience
COLOR_RED = std_msgs.msg.ColorRGBA(1.0, 0.0, 0.0, 1.0)
COLOR_GREEN = std_msgs.msg.ColorRGBA(0.0, 1.0, 0.0, 1.0)
COLOR_TRANSLUCENT = std_msgs.msg.ColorRGBA(0.0, 0.0, 0.0, 0.5)

class PickAndPlace(object):
    def __init__(self, init_node = True):
        super(PickAndPlace, self).__init__()

        # print("Class init happening bro!")
        joint_state_topic = ['joint_states:=/joint_states']

        moveit_commander.roscpp_initialize(joint_state_topic)

        # moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"

        group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=5.0)

        # See ompl_planning.yaml for a complete list
        group.set_planner_id("RRTConnect")
        # group.set_planner_id("BiEST")
        # group.set_planner_id("trajopt_interface/TrajOptPlanner")
        # group.set_planner_id("TRRT")
        group.set_max_velocity_scaling_factor(0.5)
        group.set_max_acceleration_scaling_factor(0.5)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = group.get_planning_frame()

        eef_link = group.get_end_effector_link()

        # group_names = robot.get_group_names()

        if init_node:
            print("Initializing rosnode - pnp_node")
            rospy.init_node('pnp_node', anonymous=True, disable_signals=False)
            self.remove_all_markers()

        # print(robot.get_current_state())

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.target_location_x = -100
        self.target_location_y = -100
        self.target_location_z = -100
        self.onion_color = None
        # self.onion_index = 0
        # Save some commenly used variables in the setup class
        self.ref_link = self.group.get_pose_reference_frame()
        # self.group_names = group_names
        # self.num_onions = 0
        # self.bad_onions = []
        # self.onionLoc = None
        # self.eefLoc = None
        # self.prediction = None

        # Create a publisher to visualize the position constraints in Rviz
        self.marker_publisher = rospy.Publisher(
            "/visualization_marker", visualization_msgs.msg.Marker, queue_size=20,
        )
        rospy.sleep(0.1)  # publisher needs some time to connect Rviz
        self.marker_id_counter = 0  # give each marker a unique idea

    def display_sphere(self, pose, radius=0.05, color=COLOR_GREEN):
        """ Utility function to visualize the goal pose"""

        # setup sphere marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        marker.id = self.marker_id_counter
        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.header.frame_id = self.ref_link

        # fill in user input
        marker.color = color
        marker.pose = pose
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius

        # publish it!
        self.marker_publisher.publish(marker)
        self.marker_id_counter += 1

    ## To keep things simple, we use the current end-effector pose to quickly create a reasonable goal.
    ## We also visualize the start and goal position of the end-effector in Rviz with a simple sphere.
    def create_pose_goal(self):
        self.group.clear_pose_targets()
        pose = self.group.get_current_pose()

        self.display_sphere(pose.pose, color=COLOR_RED)
        pose.pose.position.y += 0.2
        pose.pose.position.z -= 0.1

        print("Let's set the pose: \n", pose.pose)
        self.display_sphere(pose.pose)

        return pose

    ## First we create simple box constraints on the current end-effector link (:code:`self.eef_link = "gripper"`).
    def create_simple_box_constraints(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.eef_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [0.1, 0.1, 0.7]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = current_pose.pose.position.x
        cbox_pose.position.y = current_pose.pose.position.y
        cbox_pose.position.z = current_pose.pose.position.z
        cbox_pose.orientation.w = 1.0
        pcm.constraint_region.primitive_poses.append(cbox_pose)
        pcm.weight = 0.75

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm

    ## Building on the previous constraint, we can make it a line, by also reducing the dimension of the box in the x-direction.
    def create_line_constraints(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.eef_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [0.0005, 0.0005, 1.0]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = current_pose.pose.position.x
        cbox_pose.position.y = current_pose.pose.position.y
        cbox_pose.position.z = current_pose.pose.position.z
        # quat = quaternion_from_euler(pi / 4, 0, 0)
        # cbox_pose.orientation.x = quat[0]
        # cbox_pose.orientation.y = quat[1]
        # cbox_pose.orientation.z = quat[2]
        # cbox_pose.orientation.w = quat[3]
        cbox_pose.orientation.x = current_pose.pose.orientation.x
        cbox_pose.orientation.y = current_pose.pose.orientation.y
        cbox_pose.orientation.z = current_pose.pose.orientation.z
        cbox_pose.orientation.w = current_pose.pose.orientation.w
        pcm.constraint_region.primitive_poses.append(cbox_pose)
        pcm.weight = 0.75
        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm

    ## END_SUB_TUTORIAL

    def remove_all_markers(self):
        """ Utility function to remove all Markers that we potentially published in a previous run of this script. """
        # setup cube / box marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        # marker.id = 0
        # marker.type = visualization_msgs.msg.Marker.CUBE
        marker.action = visualization_msgs.msg.Marker.DELETEALL
        self.marker_publisher.publish(marker)

    def display_box(self, pose, dimensions):
        """ Utility function to visualize position constraints. """
        assert len(dimensions) == 3

        # setup cube / box marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        marker.id = self.marker_id_counter
        marker.type = visualization_msgs.msg.Marker.CUBE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.color = COLOR_TRANSLUCENT
        marker.header.frame_id = self.ref_link

        # fill in user input
        marker.pose = pose
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]

        # publish it!
        self.marker_publisher.publish(marker)
        self.marker_id_counter += 1

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

    def go_to_pose_goal(self, ox, oy, oz, ow, px, py, pz, allow_replanning=True, planning_time=10.0, thresh = 0.0025):

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
        # group.set_pose_target(pose_goal)
        group.set_joint_value_target(pose_goal, True)   # The 2nd arg is to say if approx ik is allowed
        group.allow_replanning(allow_replanning)
        group.set_planning_time(planning_time)
        # (plan, fraction) = group.compute_cartesian_path([pose_goal], 0.001, 0.0, avoid_collisions=True)
        # group.execute(plan, wait=True)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        return True

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
        ori = group.get_current_pose().pose.orientation
        status = self.go_to_pose_goal(ori.x,ori.y,ori.z,ori.w, self.target_location_x,
                                       self.target_location_y,
                                        self.target_location_z+.3,      # - 0.045,1.2114413504613049
                                       allow_replanning, planning_time, threshold)
        rospy.sleep(0.05)
        return status

    def staticDip(self, gripper_length = 0.15, tolerance=0.0055):
        
        group = self.group
        current_pose = group.get_current_pose()

        # pcm = self.create_simple_box_constraints()
        # pcm = self.create_line_constraints()
        
        # position_constraint = PositionConstraint()
        # position_constraint.target_point_offset.x = 0.1
        # position_constraint.target_point_offset.y = 0.1
        # position_constraint.target_point_offset.z = 0.1
        # position_constraint.weight = 1.0
        # position_constraint.link_name = group.get_end_effector_link()
        # position_constraint.header.frame_id = group.get_planning_frame()
        
        # orientation_constraint = OrientationConstraint()
        # orientation_constraint.orientation = current_pose.pose.orientation
        # orientation_constraint.absolute_x_axis_tolerance = 0.1
        # orientation_constraint.absolute_y_axis_tolerance = 0.1
        # orientation_constraint.absolute_z_axis_tolerance = 0.1
        # orientation_constraint.weight = 1.0
        # orientation_constraint.link_name = group.get_end_effector_link()
        # orientation_constraint.header.frame_id = group.get_pose_reference_frame()

        # constraint = Constraints()
        # constraint.orientation_constraints.append(orientation_constraint)
        # # constraint.position_constraints.append(pcm)
        # constraint.position_constraints.append(position_constraint)
        # constraint.name = "use_equality_constraints"
        # group.set_path_constraints(constraint)
        allow_replanning = True
        planning_time = 10
        dip = False
        before_dip = current_pose.pose.position.z
        print("Gripper length given: ", gripper_length)
        print("Attempting to dip to (x,y,z): ", self.target_location_x, self.target_location_y, self.target_location_z + gripper_length,)
        while not dip:
            dip = self.go_to_pose_goal(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w, self.target_location_x,  # accounting for tolerance error
                                    self.target_location_y,  
                                    self.target_location_z + gripper_length,  # For UR, eef frame is wrist_3, so we add the gripper_length to that for right amount of dip
                                    allow_replanning, planning_time, tolerance/5)
            rospy.sleep(0.01)
            print("Dip value: ", dip)
        current_pose = group.get_current_pose()
        # self.remove_all_markers()
        # group.clear_path_constraints()
        # after_dip = current_pose.pose.position.z
        # print("After dip z: ", after_dip)
        # if dip and (before_dip > after_dip):                
        #     return True
        # else:
        #     self.staticDip()
        return True


    def liftgripper(self, threshold = 0.0025):

        group = self.group
        current_pose = group.get_current_pose()

        # pcm = self.create_simple_box_constraints()
        # pcm = self.create_line_constraints()
        
        # position_constraint = PositionConstraint()
        # position_constraint.target_point_offset.x = 0.1
        # position_constraint.target_point_offset.y = 0.1
        # position_constraint.target_point_offset.z = 0.1
        # position_constraint.weight = 1.0
        # position_constraint.link_name = group.get_end_effector_link()
        # position_constraint.header.frame_id = group.get_planning_frame()
                
        # orientation_constraint = OrientationConstraint()
        # orientation_constraint.orientation = current_pose.pose.orientation
        # orientation_constraint.absolute_x_axis_tolerance = 0.1
        # orientation_constraint.absolute_y_axis_tolerance = 0.1
        # orientation_constraint.absolute_z_axis_tolerance = 0.1
        # orientation_constraint.weight = 1.0
        # orientation_constraint.link_name = group.get_end_effector_link()
        # orientation_constraint.header.frame_id = group.get_pose_reference_frame()

        # constraint = Constraints()
        # constraint.orientation_constraints.append(orientation_constraint)
        # # constraint.position_constraints.append(pcm)
        # constraint.position_constraints.append(position_constraint)
        # constraint.name = "use_equality_constraints"
        # group.set_path_constraints(constraint)
        allow_replanning = False
        planning_time = 10
        lifted = False
        z_pose = current_pose.pose.position.z + 0.12
        while not lifted:
            lifted = self.go_to_pose_goal(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w, 
                                        current_pose.pose.position.x,
                                        current_pose.pose.position.y, 
                                        z_pose,
                                        allow_replanning, planning_time, threshold)
            rospy.sleep(0.01)
            print("Lifted value: ", lifted)

        current_pose = group.get_current_pose()
        # print("Current z pose: ", current_pose.position.z)
        # self.remove_all_markers()
        # group.clear_path_constraints()
        print("Successfully lifted gripper to z: ", current_pose.pose.position.z)

        return True

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

    def goto_home(self, tolerance=0.0001, goal_tol=0.0001, orientation_tol=0.0001):

        group = self.group
        home_joint_angles = [-1.3332440853118896, -1.4162713301232834, -1.291180435811178, -1.9400388203062953, 1.6422836780548096, -0.00906402269472295]
        joint_angles = {'elbow_joint': home_joint_angles[0],
                        'shoulder_lift_joint': home_joint_angles[1],
                        'shoulder_pan_joint': home_joint_angles[2],
                        'wrist_1_joint': home_joint_angles[3],
                        'wrist_2_joint': home_joint_angles[4],
                        'wrist_3_joint': home_joint_angles[5]}
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
                rospy.sleep(0.05)
        return True

    def view(self, tol=0.001, goal_tol=0.001, orientation_tol=0.001):

        print("Attempting to reach viewpoint\n")
        group = self.group
        # view_joint_angles = [-0.6432392597198486, -2.6825577221312464, -1.0882757345782679, -0.23568494737658696, -2.2726433912860315, -4.365030709897177]
        view_joint_angles = [-0.6475113073932093, -1.9155713520445765, -1.4149178266525269, -3.124627252618307, 2.7653942108154297, 0.014964444562792778]
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
                rospy.sleep(0.05)
        return True

    def goto_bin(self, tolerance=0.001, goal_tol=0.001, orientation_tol=0.001, usePoseGoal = False):

        group = self.group
        allow_replanning = False
        planning_time = 10
        reached = False
        # print "Attempting to reach the bin"
        ori = group.get_current_pose().pose.orientation
        if usePoseGoal:

            while not reached:
                reached = self.go_to_pose_goal(ori.x,ori.y,ori.z,ori.w, 0.5, -0.08, 1, allow_replanning, planning_time, tolerance)
                rospy.sleep(0.02)

            print("Reached bin: ", reached)

        else:

            group = self.group
            bin_joint_angles = [-2.9, -2.4122820339598597, -0.51946702003479, -1.75, 1.5675392150878906, -1.4726555983172815]

            joint_angles = {'elbow_joint': bin_joint_angles[0],
                            'shoulder_lift_joint': bin_joint_angles[1],
                            'shoulder_pan_joint': bin_joint_angles[2],
                            'wrist_1_joint': bin_joint_angles[3],
                            'wrist_2_joint': bin_joint_angles[4],
                            'wrist_3_joint': bin_joint_angles[5]}
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
                    rospy.sleep(0.05)
            reached = True       
        return reached

    def goto_placeOnConv(self, tolerance=0.0001, goal_tol=0.0001, orientation_tol=0.0001):

        allow_replanning = False
        planning_time = 10
        print("Attempting to reach place on conv\n")
        group = self.group
        conv_joint_angles = [-0.400000000000, -2.350000000000, -0.460000000000, -1.90000000000, 1.60000000000, -0.030000000000]

        joint_angles = {'elbow_joint': conv_joint_angles[0],
                            'shoulder_lift_joint': conv_joint_angles[1],
                            'shoulder_pan_joint': conv_joint_angles[2],
                            'wrist_1_joint': conv_joint_angles[3],
                            'wrist_2_joint': conv_joint_angles[4],
                            'wrist_3_joint': conv_joint_angles[5]}
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
                rospy.sleep(0.05)
        return True


    def placeOnConveyor(self, tolerance=0.005):

        onConveyor = False
        allow_replanning = False
        planning_time = 5
        group = self.group
        ori = group.get_current_pose().pose.orientation
        miny = 0.25
        maxy = 0.45
        minx = -0.45
        maxx = -0.325
        yval = miny + (maxy-miny)*random.random()
        xval = minx + (maxx-minx)*random.random()
        current_pose = self.group.get_current_pose().pose
        onConveyor = self.go_to_pose_goal(ori.x,ori.y,ori.z,ori.w, xval, yval, current_pose.position.z - 0.05,
                                            allow_replanning, planning_time, tolerance)
        rospy.sleep(0.02)
        # current_pose = group.get_current_pose().pose
        # print("Current gripper pose: ", current_pose)
        print("Over the conveyor now!")
        return onConveyor

############################################## END OF CLASS ##################################################

