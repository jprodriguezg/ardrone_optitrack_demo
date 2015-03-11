#include <iostream>
#include <string>
#include <math.h> 
#include <algorithm>
#include <deque>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <optitrack_msgs/RigidBodyData.h>
#include <optitrack_msgs/RigidBodies.h>
#include <drone_control_msgs/send_control_data.h>
# define PI           3.14159265358979323846

// Define de gesturetype variable
enum gesturestype{NON_GESTURE,HOVERING,TAKEOFF,LAND,FOLLOWME};
// Some global variables
std::vector<double> gesture_marker_info (4,0), gesture_signal(2,0), leader_info; 
std::vector<float> quaternion (4,0);
std::deque<double> gestures_queue (25,NON_GESTURE);

double quaternion2angles(std::vector<float> &quaternion){
	double roll, pitch, yaw, Dyaw;
	tf::Quaternion quat(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
	tf::Matrix3x3 m(quat);
	m.getRPY(roll, pitch, yaw );
	Dyaw=(double)(yaw*180)/PI;

return Dyaw;
}


// Callbacks Definition

void hasReceivedLeaderState(const drone_control_msgs::send_control_data::ConstPtr& msg){
	
	// Obtaining leader info 
  	leader_info[0] = msg->position.x; 
	leader_info[1] = msg->position.y;
	leader_info[2] = msg->position.z;
	leader_info[3] = msg->yaw;

  return;
} 

void hasReceivedModelState(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
	// Obtaining gesture marker info 
  	gesture_marker_info[0] = msg->pose.position.x; 
	gesture_marker_info[1] = msg->pose.position.y;
	gesture_marker_info[2] = msg->pose.position.z;
	quaternion[0] = msg->pose.orientation.x;
	quaternion[1] = msg->pose.orientation.y;
	quaternion[2] = msg->pose.orientation.z;
	quaternion[3] = msg->pose.orientation.w;
	gesture_marker_info[3] = quaternion2angles(quaternion);

  return;
} 



gesturestype gesture_detector(double height){
	
	double shoulder_height, arm_longitud;
	shoulder_height = height*0,83;
	arm_longitud = height*0,48;

	// LAND CONDITION
	gesturestype gesture_out;
	if (gesture_marker_info[2] < 0.2)
		gesture_out = LAND;
	// HOVERING CONDITION
	else if(gesture_marker_info[3] >= leader_info[3]-20 && gesture_marker_info[3] <= leader_info[3]+20 && 
	gesture_marker_info[2] >= shoulder_height -0.1 && gesture_marker_info[2] <= shoulder_height + 0.1)
		gesture_out = LAND;
	// TAKEOFF FOLLWME
	else if (gesture_marker_info[2] > leader_info[3]+0.1)
		gesture_out = FOLLOWME;
	// TAKEOFF CONDITION
	else if(gesture_marker_info[3] >= leader_info[3]-110 && gesture_marker_info[3] <= leader_info[3]-70 && 
	gesture_marker_info[2] >= shoulder_height -0.1 && gesture_marker_info[2] <= shoulder_height + 0.1)
		gesture_out = LAND;
	else
		gesture_out = NON_GESTURE;

	return gesture_out;
		
}

void gestures_buffer(gesturestype gesture_detected){
	gestures_queue.pop_back();
	gestures_queue.push_front(gesture_detected);
}

double find_gesture(int begin, int end, int search_gesture){

	int count = 0;
	if (begin>end)	{	
		std::cout << "Begin value is more than End value" <<std::endl; 
		return -1;
	}
	else {
		for(int i=begin; i<=end; i++){
			if (gestures_queue[i] == search_gesture)
				count++;
			}
	}
	double out =  count*100/(begin-end+1);
	return out;
}

void drone_state(){
}

		
int main(int argc, char** argv){
    
ros::init(argc, argv, "optitrack_gestures_demo");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

std_msgs::Empty GestureMsg;
gesturestype gesture_detected = NON_GESTURE;


//Physical data
double height;
nh_.getParam("/gestures_node/leader_height",height);

ros::Subscriber optitrack_leader_sub_=nh_.subscribe("leader_topic", 1, hasReceivedLeaderState);
ros::Subscriber optitrack_gesture_marker_sub_=nh_.subscribe("gesture_marker_pose_topic", 1, hasReceivedModelState);
ros::Publisher takeoff_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
ros::Publisher land_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/land",1);

	while (ros::ok()){
	// add the gesture detected to the gestures queue
	gesture_detected = gesture_detector(height);
	gestures_buffer(gesture_detected);
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    	}

 return 0;
}
