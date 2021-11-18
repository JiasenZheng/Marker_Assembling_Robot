# final-project-group-4-inc

# Dependencies

## SMACH
### Installing and using SMACH-ROS
* run the following command in a terminal: `sudo apt-get install ros-noetic-smach ros-noetic-smach-ros ros-noetic-executive-smach ros-noetic-smach-viewer`
* smach is out of date and not maintained properly but most of the files can be fixed with a single key press.
#### Smach_viewer
* if you followed the instructions for installaition then `smach_viewer.py` should be located here: `/opt/ros/(ros-version)/lib/smach_viewer`. For example I would find the file in `/opt/ros/noetic/lib/smach_viewer`.
* You need to edit `smach_viewer.py` but only root has writing permission for the file. If you would like to edit the file without changing its permissions open the file with this command: `sudo gedit smach_viewer.py`.
* There is a shabang on line one of the file (`#!/user/bin/env python`). Change it to: `#!/user/bin/env python3`
* Please hold off on doing this.  There is a real posibility that we can't use smach_viewer.
#### Running Task master
* To run task master simply run: `rosrun group4 TaskMaster`