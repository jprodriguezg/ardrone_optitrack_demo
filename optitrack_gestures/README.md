#optitrack_gestures

Handles the state machine which control each step of the demostration. Also, it includes the gestures definitions to perform the interaction between the drone and the users. As the control node, the package has several private parameters which should be defined before start the demo.

* `leaders_heights`
* `leaders_id` These ids are defined in the Optitrack system. Please check it and copy them here.
* `mission_target` Coordinates of the place where the drone should go to perform the object detection mission
* `delta_pose_vision` It is the pose which the drone will take if some object is detected.
