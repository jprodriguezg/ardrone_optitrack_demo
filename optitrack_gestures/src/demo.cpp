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
#include <drone_control_msgs/demo_info.h>
#include <drone_control_msgs/drone_control_info.h>
# define PI           3.14159265358979323846

drone_control_msgs::drone_control_info publish_data;

// Defining de gesturetype and drone_state varibles
enum gesturestype{NON_GESTURE,HOVERING_GESTURE,TAKEOFF_GESTURE,LAND_GESTURE,FOLLOWME_GESTURE};
enum drone_state{LANDED,HOVERING,FOLLOWING_LEADER};

// Some global variables
std::vector<double> gesture_marker_info (4,0), leader_info(4,0), LeaderFrame(2,0), GestureMarkerFrame(2,0); 
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

	publish_data.position.x = leader_info[0];
	publish_data.position.y = leader_info[1];
	publish_data.position.z = leader_info[2];
	publish_data.yaw = leader_info[3];

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

	// Publishing info 
	publish_data.targetYaw = gesture_marker_info[3];
	publish_data.target.x = gesture_marker_info[0];
	publish_data.target.y = gesture_marker_info[1];
	publish_data.target.z = gesture_marker_info[2];

  return;
} 

// Functions
void WorldToLeaderframe(double angle){

	angle=(PI*angle)/180;
	LeaderFrame[0]=cos(angle)*leader_info[0]+sin(angle)*leader_info[1];
	LeaderFrame[1]=-sin(angle)*leader_info[0]+cos(angle)*leader_info[1];
	GestureMarkerFrame[0]=cos(angle)*gesture_marker_info[0]+sin(angle)*gesture_marker_info[1];
	GestureMarkerFrame[1]=-sin(angle)*gesture_marker_info[0]+cos(angle)*gesture_marker_info[1];

	publish_data.framePosition.x=LeaderFrame[0];
	publish_data.framePosition.y=LeaderFrame[1];
	publish_data.framePosition.z=leader_info[2];
	publish_data.frameTarget.x=GestureMarkerFrame[0];
	publish_data.frameTarget.y=GestureMarkerFrame[1];
	publish_data.frameTarget.z=gesture_marker_info[2];
}

gesturestype gesture_detector(double height){
	
	double shoulder_height, arm_longitud;
	//shoulder_height = height*(7/8);
	//arm_longitud = height*(4/9);
	shoulder_height = 1.55;
	arm_longitud = 0.6;
	gesturestype gesture_out;

	WorldToLeaderframe(leader_info[3]);

	// LAND CONDITION
	if (gesture_marker_info[2] < 0.2)
		gesture_out = LAND_GESTURE;
	// HOVERING CONDITION (altitude, x position, y position)
	else if(gesture_marker_info[2] >= shoulder_height -0.2 && gesture_marker_info[2] <= shoulder_height + 0.2 &&
		GestureMarkerFrame[0] >= LeaderFrame[0]-arm_longitud-0.2 && GestureMarkerFrame[0] <= LeaderFrame[0]-arm_longitud+0.2 &&
		GestureMarkerFrame[1] >= LeaderFrame[1]-0.1 && GestureMarkerFrame[1] <= LeaderFrame[1]+0.1)
		gesture_out = HOVERING_GESTURE;
	// TAKEOFF FOLLOWME
	else if (gesture_marker_info[2] > leader_info[2]+0.1)
		gesture_out = FOLLOWME_GESTURE;
	// TAKEOFF CONDITION (altitude, y position)
	else if(gesture_marker_info[2] >= shoulder_height -0.2 && gesture_marker_info[2] <= shoulder_height + 0.2 &&
		GestureMarkerFrame[0] >= LeaderFrame[0]-0.15 && GestureMarkerFrame[0] <= LeaderFrame[0]+0.15 &&
		GestureMarkerFrame[1] >= LeaderFrame[1]+arm_longitud-0.2 && GestureMarkerFrame[1] <= LeaderFrame[1]+arm_longitud+0.2)
		gesture_out = TAKEOFF_GESTURE;
	else
		gesture_out = NON_GESTURE;

	return gesture_out;
		
}

void gestures_buffer(gesturestype gesture_detected){
	// This deque works from the front to back (0 to 24)
	gestures_queue.pop_back();
	gestures_queue.push_front(gesture_detected);
}

double find_gesture(int begin, int end, gesturestype search_gesture){

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

void drone_status(drone_state &current_state){

	switch (current_state) {
		case LANDED:  
			if (find_gesture(0,9,TAKEOFF_GESTURE)>=90)
				current_state = HOVERING;
			else
				current_state = LANDED;  
			break;
		case HOVERING:  
			if (find_gesture(0,9,FOLLOWME_GESTURE)>=90)
				current_state = FOLLOWING_LEADER;
			else if (find_gesture(0,9,LAND_GESTURE)>=90)
				current_state = LANDED;
			else
				current_state = HOVERING;
			break;
		case FOLLOWING_LEADER: 
			if (find_gesture(0,9,HOVERING_GESTURE)>=90)
				current_state = HOVERING;
			else if (find_gesture(0,9,LAND_GESTURE)>=90)
				current_state = LANDED;
			else
				current_state = FOLLOWING_LEADER;
			break;
	  }
}

int main(int argc, char** argv){
    
ros::init(argc, argv, "optitrack_gestures_demo");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

// Map declarations
std::map<gesturestype,std::string> gestures;
	// Map definition
	gestures[NON_GESTURE]="non_gesture_detected";
	gestures[HOVERING_GESTURE] = "hovering";
	gestures[TAKEOFF_GESTURE]="takeoff";
    	gestures[LAND_GESTURE]="land";
    	gestures[FOLLOWME_GESTURE]="follow_me";

std::map<drone_state,std::string> status;
	// Map definition 
	status[LANDED]="landed";
	status[HOVERING] = "hovering";
	status[FOLLOWING_LEADER]="following_leader";

// Variablas declaration 
std_msgs::Empty GestureMsg;
drone_control_msgs::demo_info data_out;
gesturestype gesture_detected = NON_GESTURE;
drone_state current_state = LANDED;
//Physical user data
double height;
nh_.getParam("/gestures_node/leader_height",height);

// Publishers and subscribers
ros::Subscriber optitrack_leader_sub_=nh_.subscribe("leader_pose_topic", 1, hasReceivedLeaderState);
ros::Subscriber optitrack_gesture_marker_sub_=nh_.subscribe("gesture_marker_pose_topic", 1, hasReceivedModelState);
ros::Publisher demo_info_pub_=nh_.advertise<drone_control_msgs::demo_info>("demo_info_topic",1);
ros::Publisher takeoff_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
ros::Publisher land_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
ros::Publisher drone_info_pub_=nh_.advertise<drone_control_msgs::drone_control_info>("/demo_bodies_info", 1);

	
	// Main loop
	while (ros::ok()){
	// add the gesture detected to the gestures queue
	gesture_detected = gesture_detector(height);
	gestures_buffer(gesture_detected);
	drone_status(current_state);

	// Publishing process
	data_out.gesture_detected = gestures[gesture_detected];
	data_out.demo_status = status[current_state];
	demo_info_pub_.publish(data_out);
	drone_info_pub_.publish(publish_data);

   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    	}

 return 0;
}
