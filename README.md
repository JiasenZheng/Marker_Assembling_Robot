# Group4 - Picking, placing, and cappings markers and caps

The goal of this project was to pick, place and cap markers with caps, and was thus heavily inspired by the application of robots in manufacturing and industry. Our project used a RealSense camera to detect colors of the markers, and MoveIt manipulation commands to actuate the robot. Franka-specific actions also were used to grip caps and markers during movement. The framework of the project was controlled using a state machine developed in the ROS package called SMACH. 

## Instructions to run the robot

The robot is run by issuing the following set of commands. To start, the user must connect to the Franka robot and enable ROS by activating the FCI.

The subsequent seuqence of steps are:
1) SSH into the robot using: 
`ssh -oSendEnv=ROS_MASTER_URI student@station`



2) Launch the the franka ros controller using the following command in the SSH terminal:
`roslaunch panda_moveit_config panda_control_moveit_rviz.launch launch_franka_control:=false robot_ip:=robot.franka.de`


3) Up the collision limits for the robot by calling the following node and service:
`rosrun group4 limit_set`
`rosservice call /coll_hi`


4) Launch the robot manipulation and vision commands using the following launch file:
`roslaunch group4 launch_robot.launch`


5) Run the state machine to initiate the pick and place sequence using the following command:
`rosrun group4 TaskMaster`



## Subsystems 

### Manipulation

The manipulation package relies on several different nodes in order to function:
1) manipulation_cap provides low level position and orientation sensing services, along with error recovery, movements and gripper grasping
2) manipulation_macro_a provides position movement services for image captures using the realsense
3) manipulation_press provides a pressing service to cap the markers
4) manipulation_local provides manipulation services for moving in between trays
5) manipulation_pnp provides pick and place services between the feed and assembly trays
6) debug_manipulation logs the external forces experienced by the robot

### Vision


### SMACH
#### Installing and using SMACH-ROS
* run the following command in a terminal: `sudo apt-get install ros-noetic-smach ros-noetic-smach-ros ros-noetic-executive-smach ros-noetic-smach-viewer`
* smach is out of date and not maintained properly but most of the files can be fixed with a single key press.
#### Smach_viewer
* This package uses its own version of smach_viewer and is a node within the package.
#### Running Task master
* To run task master simply run: `rosrun group4 TaskMaster`

# Video demo
* The user interface of processed images and Franka arm visualizations on Rviz:   https://youtu.be/oCTd5CoBUqM
* The side view of the video record of Franka arm assembling the markers (3.0X faster): https://youtu.be/m37ZtrH2SsE
