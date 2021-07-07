# ur3e_irl_project

Author: Prasanth Sengadu Suresh.

Owned By: THINC Lab, Department of Computer Science,
          University of Georgia.

Currently Managed By: Prasanth Sengadu Suresh.

This package uses Inverse Reinforcement Learning -- Learning from Demonstration to teach UR3e Robot to perform vegetable sorting on a conveyor line alongside a Human Expert.

This package is built upon the UR ROS packages and uses OnRobot SG Gripper as the End Effector.

## The following instructions are written for Ubuntu 20.04, ROS Noetic. If you are on Melodic, make sure you install the appropriate melodic branches/versions of the required packages.

If you need to install the packages for Kinect V2 with Ubuntu, check out this [link](https://github.com/thinclab/sawyer_irl_project/blob/master/Kinect_install_readme.md).

If you need to install the packages for Realsense2-D435 with Ubuntu, check out this [link](https://github.com/thinclab/sawyer_irl_project/blob/master/Realsense_install_readme.md).

The following are the steps to be followed to get this package working:

  Assuming you have a working version of Ubuntu (This package has been built and tested on Ubuntu 18.04 and 20.04)
  
  1.) Install ROS
  
   [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
      
   [Catkin Workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)
      
   [Moveit Install](https://moveit.ros.org/install/)
   
   [Moveit Workspace Setup](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
   
   Also install:
   
          sudo apt install ros-<YOUR ROS DISTRO>-ros-controllers
   
  2.) We need an upgraded IK solver for smooth working of Sawyer:
  
   - Use the following command:
   
     `sudo apt-get install ros-<YOUR-ROS-DISTRO>-trac-ik-kinematics-plugin`
     
   - Then you need to find the kinematics.yaml file within sawyer_moveit_config folder and update this line: 
   `kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin` to `kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`
  
   - Here's the wiki [link](https://ros-planning.github.io/moveit_tutorials/doc/trac_ik/trac_ik_tutorial.html) for reference.

   - After this, in the same kinematics.yaml file, add this at the end of each move group (in this case, right_arm and head):
  
          solve_type: Distance
   
     This makes the IK solver prefer least path cost solutions.
   
  2.1) This is still a work in progress, but if you want to add STOMP path planning library as a smoothing filter over OMPL, check these [instructions](https://github.com/thinclab/sawyer_irl_project/blob/master/OMPL-STOMP_smoothing_filter.md)
      
  3.) Now that you have a catkin workspace setup, in you src folder, git clone the following packages:
  
   - These packages have changes that are not a part of their default branches. Make sure you clone them from the links below.

          git clone https://github.com/thinclab/roboticsgroup_gazebo_plugins-1.git
      
          git clone https://github.com/pal-robotics/gazebo_ros_link_attacher/tree/melodic-devel.git
      
          git clone https://github.com/thinclab/kinect_v2_udrf.git
          
          git clone https://github.com/thinclab/velocity_plugin.git
          
          git clone https://github.com/thinclab/iai_kinect2.git
          
          git clone https://github.com/thinclab/sanet_onionsorting.git
          
          git clone https://github.com/thinclab/Universal_Robots_ROS_Driver.git
          
          git clone https://github.com/thinclab/ur3e_irl_project.git
          
          git clone https://github.com/gavanderhoorn/industrial_robot_status_controller.git
          
          git clone https://github.com/ros-industrial/ur_msgs.git
          
          git clone https://github.com/fmauch/universal_robot.git
          
          git clone https://github.com/ros-industrial/industrial_core.git
          
   - Use the following command to update all your packages and drivers:
   
          sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade

   - cd into catkin_ws and install all dependencies for these packages: 

          rosdep install --from-paths src --ignore-src --rosdistro=<YOUR ROS DISTRO> -y -i --verbose

   - If you see any uninstalled dependencies, you might have to manually install them using apt-get install or pip install.
   - If you still have errors, use 

         rospack depends [name of package]

   - This should give you the dependencies you are missing for any given package.
   - Do a 'catkin_make_isolated' to compile.

  3.) You are almost ready to run the simulation. Double check if you have installed all the required plugins for moveit (esp moveit controllers)
  
   - Add this line to the end of your ~/.bashrc file(running it on terminal or copying it to bashrc without the 'export' part doesn't work. Copy it as it is.): 
   
         export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/ur3e_irl_project/meshes:$GAZEBO_MODEL_PATH
         
  4.) To work on the real robot, make sure teh robot is in remote control mode on the control pendant, then ensure the correct ip address of your computer has been added to the pendant. Also ensure that the static ip of the robot has been correctly added to robot.launch file inside ur3e_irl_project/launch.
  
  5.) Now run:
  
        roslaunch ur3e_irl_project robot.launch
