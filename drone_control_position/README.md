#drone_control_position

Package in charge of Drone's control position. The package has several parameters to modify the behavior of the internal P or PD controllers (altitude, pitch, roll and yaw). Also, some parameters handle the safe restrictions of the demo.

* `Kd` and `Kp` Correspond to the diferential and proportional gains of each controller.
* `velocity_limit`  Defines the maximum velocities of each control action.
* `delta_pose vector`  Defines the distance (if it's necessary) between the target point and the Drone
* `virtual_fence`  Represents the area where the drone can fly. If the drone goes out of the fence, it will enter in emergency mode by landing or turning off the propellers.
