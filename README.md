# ardrone_optitrack_demo

In this repository are the packages which belong to the demo project.
The demo includes the interaction between the AR Drone 2.0 and the Users. The interactions are perform by using the Optritrack system and ROS. 

Here is a little resume of each package included in the project

* drone_control_position:
Package in charge of Drone's control position in the demo. The package has several parameters  definded to modify the behavior of the internal P or PD controllers (altitude, pich, roll and yaw). Also between the parameter are defined the safety restrictions for the demo. 

 * Kd and Kp: Correspond to the differentian and proportinal gains of each controller.
 * velocity_limit: Defines the maximum velocities of each control action in the demo.
 * delta_pose vector: Defines the distance (if it's necessary) between the target point and the Drone
 * virtual_fence: Represents the area where the drone can fly. If the drone goes out of the fence, it wil enter in emergency mode by landing or turning off the propellers.

* drone_following_leader:
Includes the nodes in charge of define the position of the leader to follow (e.g. The pose of the leader 1 o 2. If any leader is not detected the drone must mantain its current pose). It also restricts the target points values which drone can follow. 

* messages:
contains all the messages of the demo project

* drone_feedback signals:
The package includes the package to start the feedback sounds server and the node which controls the led feedbacks of the drone. For the sound server is necessary to define several parameters with the location of the audio files to play.

* send_control_position:
It is a simple package created to define directly the target points to follow via ROS parameters.

* visual_object_detector:
The package is in charge of peform the visual detection task during the demo. It creates the output_image topic (It should be remped) where the last image processed is show.

* optitrack_gestures:
Handles the state machine which control each step of the demostration. Also, it includes the gestures definitions to perform the interaction between the drone and the userw. As the control node, the package has several parameters which should be defined before start the demo.
 * leaders_heights
 * leaders_id: These ids are defined in the optitrack system. Please check it and copy them here.
 * mission_target: Coordinates of the place where the drone should go to perform the object detection mission
 * delta_pose_vision: It is the pose which the drone will go if some object is detected.

## Run the demo

* Optitrack 
 * Before starting the demo, the user should calibrate the optitrack to obtain a high accuarte mesurement of each rigid body of the system. 
 * Rigid  bodies definition: To perform the demo 5 objects must be defined: drone, user 1, user 2, gesture marker 1, and gesture marker 2. It's too IMPORTANT to define the rigid body of each user cap in front of the drone face. If the definition is wrong, the drone will follow the leader in an incorrect angle. (e.g. The drone could follow the back of the user)
