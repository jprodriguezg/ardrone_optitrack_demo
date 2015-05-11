# ardrone_optitrack_demo

In this repository are the packages which belong to the demo project.
The demo includes the interaction between the AR Drone 2.0 and the Users. The interactions are perform by using the Optritrack system and ROS. 

Here is a small description of each package included in the project

* drone_control_position:
Package in charge of Drone's control position in the demo. Several parameters should be definded in order to guarantee the correct behavior of the internal PD controllers (altitude, pich, roll and yaw). Also the safety restrictions for the demo are defined in the node. 

 * Kd and Kp vectors correspond to the differentian and proportinal gains of the controller.
 *velocity_limit is a vector which defines the maximum velocities for each control action in the demo.
 *The delta_pose vector corresponds to the distance which the drone should mainten between it and the target point.   *This variable is necessary to be defined if the drone is going to follow and user.


