# Group4 - Picking, placing, and cappings markers and caps

The goal of this project was to pick, place and cap markers with caps, and was thus heavily inspired by the application of robots in manufacturing and industry. Our project used a RealSense camera to detect colors of the markers, and MoveIt manipulation commands to actuate the robot. Franka-specific actions also were used to grip caps and markers during movement. The framework of the project was controlled using a state machine developed in the ROS package called SMACH. 

## Instructions to run the robot

The robot is run by issuing the following set of commands. To start, the user must connect to the Franka robot and enable ROS by activating the FCI.

The subsequent seuqence of steps are:
1) SSH into the robot using: 
`ssh -oSendEnv=ROS_MASTER_URI student@station`



2) Launch the the franka ros controller using the following command in the SSH terminal:
`roslaunch panda_moveit_config panda_control_moveit_rviz.launch launch_franka_control:=false robot_ip:=robot.franka.de`



3) Launch the robot manipulation and vision commands using the following launch file:
`roslaunch group4 launch_robot.launch`


4) Run the state machine to initiate the pick and place sequence using the following command:
`rosrun group4 TaskMaster`

### Automated Startup
Incase you want a faster startup method a bash script is set up which at least creates the terminals necessary for start up.
1) start by running the command: `bash sourceWS`
- This will source the workspace directory in your .bashrc
2) Run this command to begin: `bash start`
- This should start up every process to get the project started.  If it doesn't please refer to the section above.
3) When you are done using the start script run this command: `bash wsSourceRemove`
- This will remove the workspace from your .bashrc file, **only run once**.

## Subsystems 

### Manipulation

The manipulation

### Vision


### SMACH

* If you are interested in editing or changing the behavior we encourage you take a look at SMACHs tutorial at: http://wiki.ros.org/smach

#### Installing and using SMACH-ROS
* run the following command in a terminal: `sudo apt-get install ros-noetic-smach ros-noetic-smach-ros ros-noetic-executive-smach`

#### Running Task master
* To run task master simply run: `rosrun group4 TaskMaster`

# Video demo
* The user interface of processed images and Franka arm visualizations on Rviz:   https://youtu.be/oCTd5CoBUqM
* The side view of the video record of Franka arm assembling the markers (3.0X faster): https://youtu.be/m37ZtrH2SsE
