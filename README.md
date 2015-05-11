# ardrone_optitrack_demo

In this repository are the packages of the demo project.
The demo includes the interaction between the AR Drone 2.0 and 2 users. The interactions are perform by using the Optitrack system and ROS.

Here is a little resume of each package included in the project

* drone_control_position:
Package in charge of Drones control position. The package has several parameters to modify the behavior of the internal P or PD controllers (altitude, pitch, roll and yaw). Also, some parameters handle the safe restrictions of the demo.

 * Kd and Kp: Correspond to the diferential and proportional gains of each controller.
 * velocity_limit: Defines the maximum velocities of each control action.
 * delta_pose vector: Defines the distance (if it's necessary) between the target point and the Drone
 * virtual_fence: Represents the area where the drone can fly. If the drone goes out of the fence, it will enter in emergency mode by landing or turning off the propellers.

* drone_following_leader:
Includes the nodes in charge of define the position of the leader to follow (e.g. The pose of the leader 1 o 2). If any leader is not detected, the drone must maintain its current pose. The nodes also restrict the target points values which drone can follow.

* messages:
contains all the messages of the demo project

* drone_feedback signals:
The package includes  a node to start the feedback sounds server and  another  node to control the led feedbacks. Before run the sound server node, please define the parameters with the location of the audio files to play.

* send_control_position:
It is a simple package created to define directly the target points to follow via ROS parameters.

* visual_object_detector:
The package is in charge of perform the visual detection task during the demo. It creates the output image topic where the last image processed is show.

* optitrack_gestures:
Handles the state machine which control each step of the demostration. Also, it includes the gestures definitions to perform the interaction between the drone and the users. As the control node, the package has several parameters which should be defined before start the demo.
 * leaders_heights
 * leaders_id: These ids are defined in the Optitrack system. Please check it and copy them here.
 * mission_target: Coordinates of the place where the drone should go to perform the object detection mission
 * delta_pose_vision: It is the pose which the drone will take if some object is detected.

## Before running the demo

* Optitrack 
 * Before starting the demo, the user should calibrate the optitrack to obtain an accurate measure of the pose of each rigid body. 
 * Rigid bodies definition: To perform the demo 5 objects must be defined: drone, user 1, user 2, gesture marker 1, and gesture marker 2. It's too IMPORTANT to define the rigid bodies of the users caps in front of the drone face. If the definition is wrong, the drone will follow the leader in an incorrect angle (e.g. The drone could follow the back of the user).
 
* launch files 
 * demo.launch: In this file are defined all the parameters named above. The user should check and modify (If it's necessary) each parameter of file in order to run correctly all the nodes. Please, be careful with definition of parameters and remapping topics (e.g. "marker_leader_1_pose_topic" which contains the pose of the marker 1). If these are wrong defined, the nodes will work incorrectly.
 
## Run the demo
* After connect your computer to the AR Drone 2.0, run the ardrone_demo launch file. 
* Start the demo nodes by running the demo launch file
* Enjoy
