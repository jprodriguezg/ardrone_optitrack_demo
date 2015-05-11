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

* messages
Pacakge where all the messages of the system are defined

* drone_feedback signals
The package includes the package to start the feedback sounds server and the node which controls the led feedbacks of the drone. For the sound server is necessary to define several parameters with the location of the audio files to play.

* send_control_position
It is a small package created to send directly the target points to follow via ROS parameters.


