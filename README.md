# ardrone_optitrack_demo

In this repository are the packages of the Demo project.

The Demo includes the interaction between the AR Drone 2.0 and 2 users. The interactions are perform by using the Optitrack system and ROS.

The general information of packages is in the README file of each folder.

## Before running the demo

* Optitrack 
 * Before starting the demo, the user should calibrate the optitrack to obtain an accurate measure of the pose of each rigid body. 
 * Rigid bodies definition: To perform the demo 5 objects must be defined: drone, user 1, user 2, gesture marker 1, and gesture marker 2. It's too IMPORTANT to define the rigid bodies of the users caps in front of the drone face. If the definition is wrong, the drone will follow the leader in an incorrect angle (e.g. The drone could follow the back of the user).
 
* Comunication between Demo and Optitrack computers
  Both machines need to be connected on the same network. One of the computers should be defined as Master in order handle the nodes, servers and parameters of ROS. In the `windows_scripts` folder are some examples of how to define the Optitrack computer as Master.
 
* Main launch file
 * `demo.launch` (demo folder): In this file are defined all the parameters of Demo nodes. The user should check and modify (If it's necessary) each parameter in order to run correctly all the nodes. Please, be careful with definition of parameters and remapping topics (e.g. "marker_leader_1_pose_topic" which contains the pose of the marker 1). If these are wrong defined, the system will work incorrectly.
 
## Run the demo
1. After connect your computer to the AR Drone 2.0, run the ardrone driver demo  file: `roslaunch optitrack_gestures ardrone_demo.launch` 
2. Start the demo: `roslaunch demo demo.launch` 
3. Enjoy
