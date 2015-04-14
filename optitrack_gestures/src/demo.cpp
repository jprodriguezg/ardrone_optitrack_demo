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
#include <ardrone_autonomy/Navdata.h>
#include <optitrack_msgs/RigidBodyData.h>
#include <optitrack_msgs/RigidBodies.h>
#include <drone_control_msgs/send_control_data.h>
#include <drone_control_msgs/demo_info.h>
#include <drone_control_msgs/drone_control_info.h>
#include <visual_object_detector/DetectObject.h>
#include <ardrone_autonomy/CamSelect.h>
# define PI           3.14159265358979323846

// Defining de gesturetype and drone_state varibles
enum gesturestype{NON_GESTURE,HOVERING_GESTURE,TAKEOFF_GESTURE,LAND_GESTURE,FOLLOW_LEADER_1_GESTURE,FOLLOW_LEADER_2_GESTURE,MISSION_GESTURE};
enum drone_state{LANDED,HOVERING,FOLLOWING_LEADER,MISSION,EMERGENCY};

// Some global variables
std::vector<double> drone_info (4,0), leader1_marker_info (4,0), leader2_marker_info (4,0),leader_info(4,0), LeaderFrame(2,0), TargetControlFrames(4,0), GestureMarkerFrame(2,0), mission_pose (4,0), dPose (4,0); 
std::vector<float> quaternion1 (4,0), quaternion2 (4,0);
std::vector<float> leaders_id(2,0);  // Posible leaders ids
std::deque<gesturestype> gestures_queue (40,NON_GESTURE);
int leader_id = 0, detector_flag = 0;   // Global variable of leader
double VxDrone, VyDrone;
std::string drone_control_status;

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


void hasReceivedMarkerLeader1State(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
	// Obtaining gesture marker info 
  	leader1_marker_info[0] = msg->pose.position.x; 
	leader1_marker_info[1] = msg->pose.position.y;
	leader1_marker_info[2] = msg->pose.position.z;
	quaternion1[0] = msg->pose.orientation.x;
	quaternion1[1] = msg->pose.orientation.y;
	quaternion1[2] = msg->pose.orientation.z;
	quaternion1[3] = msg->pose.orientation.w;
	leader1_marker_info[3] = quaternion2angles(quaternion1);

  return;
} 


void hasReceivedMarkerLeader2State(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
	// Obtaining gesture marker info 
  	leader2_marker_info[0] = msg->pose.position.x; 
	leader2_marker_info[1] = msg->pose.position.y;
	leader2_marker_info[2] = msg->pose.position.z;
	quaternion2[0] = msg->pose.orientation.x;
	quaternion2[1] = msg->pose.orientation.y;
	quaternion2[2] = msg->pose.orientation.z;
	quaternion2[3] = msg->pose.orientation.w;
	leader2_marker_info[3] = quaternion2angles(quaternion2);

  return;
} 


void hasReceivedDroneControlState(const drone_control_msgs::drone_control_info::ConstPtr& msg){
	
	drone_control_status = msg->mode;
	drone_info[0] = msg->position.x; 
	drone_info[1] = msg->position.y;
	drone_info[2] = msg->position.z;
	drone_info[3] = msg->yaw;

	TargetControlFrames[0] = msg->framePosition.x;
	TargetControlFrames[1] = msg->framePosition.y;

	TargetControlFrames[0] = msg->frameTarget.x;
	TargetControlFrames[1] = msg->frameTarget.y;

  return;
} 


void hasReceivedNavdataInfo(const ardrone_autonomy::NavdataConstPtr msg){
	VxDrone = msg->vx;
	VyDrone = msg->vy;
} 

// Functions
void WorldToLeaderframe(double angle){
	
	angle=(PI*angle)/180;
	LeaderFrame[0] = cos(angle)*leader_info[0]+sin(angle)*leader_info[1];
	LeaderFrame[1] = -sin(angle)*leader_info[0]+cos(angle)*leader_info[1];
	if (leaders_id[0] == leader_id){	// Creates a new leader 1 frame
		GestureMarkerFrame[0] = cos(angle)*leader1_marker_info[0]+sin(angle)*leader1_marker_info[1];
		GestureMarkerFrame[1] = -sin(angle)*leader1_marker_info[0]+cos(angle)*leader1_marker_info[1];
	}
	else if (leaders_id[1] == leader_id){	// Creates a new leader 2 frame
		GestureMarkerFrame[0] = cos(angle)*leader2_marker_info[0]+sin(angle)*leader2_marker_info[1];
		GestureMarkerFrame[1] = -sin(angle)*leader2_marker_info[0]+cos(angle)*leader2_marker_info[1];
	}
}

gesturestype gesture_detector(std::vector<double> heights, int id){

	double shoulder_height = 1.55, arm_longitud = 0.6, shoulder_longitude = 0.25;  // For a person with 1.80 m of height
	gesturestype gesture_out;
	double current_leader_marker_altitude = 0;

	if (id == leaders_id[0]){
		current_leader_marker_altitude = leader1_marker_info[2];
		shoulder_height = heights[0]*(double(7)/8);
		arm_longitud = heights[0]*(double(3)/8);
		shoulder_longitude = heights[0]*(double(2)/15);
		}
	else if (id == leaders_id[1]){
		current_leader_marker_altitude = leader2_marker_info[2];
		shoulder_height = heights[1]*(double(7)/8);
		arm_longitud = heights[1]*(double(3)/8);
		shoulder_longitude = heights[1]*(double(2)/15);
		}
	
	//std::cout << "Shoulder "<<shoulder_height<<" arm  "<<arm_longitud<<std::endl;
	// Transform the leader and marker frames base on the angle of the leader
	WorldToLeaderframe(leader_info[3]);

	// LAND CONDITION  -- Controlled by both leaders
	if (leader1_marker_info[2] < 0.2 || leader2_marker_info[2] < 0.2)
		gesture_out = LAND_GESTURE;
	// FOLLOWME CONDITION	-- Controlled by both leaders
	else if (leader1_marker_info[2] > leader_info[2]+0.1)
		gesture_out = FOLLOW_LEADER_1_GESTURE;		
	else if (leader2_marker_info[2] > leader_info[2]+0.1)
		gesture_out = FOLLOW_LEADER_2_GESTURE;
	// TAKEOFF CONDITION (altitude, y position) -- Only given by leader 1
	else if(/**id == leaders_id[0] &&*/ leader1_marker_info[2] >= shoulder_height -0.1 && leader1_marker_info[2] <= shoulder_height+0.1 && GestureMarkerFrame[0] >= LeaderFrame[0]-0.15 && GestureMarkerFrame[0] <= LeaderFrame[0]+0.15 && 
		GestureMarkerFrame[1] >= LeaderFrame[1]+arm_longitud-0.2 && GestureMarkerFrame[1] <= LeaderFrame[1]+arm_longitud+0.2)
		gesture_out = TAKEOFF_GESTURE;
	//HOVERING CONDITION (altitude, x position, y position) -- Controlled by both leaders
	else if(current_leader_marker_altitude >= shoulder_height -0.1 && current_leader_marker_altitude <= shoulder_height + 0.1 &&
		GestureMarkerFrame[0] >= LeaderFrame[0]-arm_longitud-0.2 && GestureMarkerFrame[0] <= LeaderFrame[0]-arm_longitud+0.2 && GestureMarkerFrame[1] >= LeaderFrame[1]-0.1 && GestureMarkerFrame[1] <= LeaderFrame[1]+0.1)
		gesture_out = HOVERING_GESTURE;
	// MISSION CONDITION (altitude, x position, y position) -- Controlled by both leaders
	else if(current_leader_marker_altitude >= shoulder_height -0.1 && current_leader_marker_altitude <= shoulder_height + 0.1 &&
		GestureMarkerFrame[0] >= LeaderFrame[0]-0.1 && GestureMarkerFrame[0] <= LeaderFrame[0]+0.1 && GestureMarkerFrame[1] >= LeaderFrame[1]-shoulder_longitude-0.1 && GestureMarkerFrame[1] <= LeaderFrame[1]-shoulder_longitude+0.1)
		gesture_out = MISSION_GESTURE;
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
	double out =  count*double(100)/(end-begin+1);
	return out;
}

void drone_status(drone_state &current_state, ros::Publisher &takeoff, ros::Publisher &land, ros::NodeHandle &nh_, ros::ServiceClient &detector_client, visual_object_detector::DetectObject &detector_srv, ros::ServiceClient &drone_cam_client, ardrone_autonomy::CamSelect &drone_cam_srv){

	std_msgs::Empty EmptyMsg;
	switch (current_state) {
		case LANDED:  
			if (drone_control_status == "Emergency")
				current_state = EMERGENCY;
			else if (find_gesture(0,20,TAKEOFF_GESTURE)>=90){
				current_state = HOVERING;
				takeoff.publish(EmptyMsg);
				}
			else{
				current_state = LANDED;  
				leader_id = leaders_id[0];  // Change the global id of the leader to leader_1
				nh_.setParam("/leader_selector/leader_id",leader_id);   // Change the global leader id
				}
			break;

		case HOVERING:  
			if (drone_control_status == "Emergency")
				current_state = EMERGENCY;
			else if (find_gesture(0,20,FOLLOW_LEADER_1_GESTURE)>=90){
				current_state = FOLLOWING_LEADER;
				leader_id = leaders_id[0];  // Change the global id of the leader to leader_1
				nh_.setParam("/leader_selector/leader_id",leader_id);   // Change the global leader id
				}
			else if (find_gesture(0,20,FOLLOW_LEADER_2_GESTURE)>=90){
				current_state = FOLLOWING_LEADER;
				leader_id = leaders_id[1]; 	// Change the global id of the leader to leader_2
				nh_.setParam("/leader_selector/leader_id",leader_id);   // Change the global leader id
				}
			else if (find_gesture(0,20,LAND_GESTURE)>=90){
				current_state = LANDED;
				land.publish(EmptyMsg);
				}
			else
				current_state = HOVERING;
			break;

		case FOLLOWING_LEADER: 
			if (drone_control_status == "Emergency")
				current_state = EMERGENCY;
			else if (find_gesture(0,20,HOVERING_GESTURE)>=90)
				current_state = HOVERING;
			else if (find_gesture(0,20,LAND_GESTURE)>=90){
				current_state = LANDED;
				land.publish(EmptyMsg);
				}
			else if (find_gesture(0,20,MISSION_GESTURE)>=90){
				current_state = MISSION;
				drone_cam_client.call(drone_cam_srv);
				}
			else
				current_state = FOLLOWING_LEADER;
			break;

		case MISSION: 
			if (drone_control_status == "Emergency")
				current_state = EMERGENCY;
			else if (TargetControlFrames[0] >= TargetControlFrames[2]-0.05 && TargetControlFrames[0] <= TargetControlFrames[2]+0.05 && TargetControlFrames[1] >= TargetControlFrames[3]-0.1 && TargetControlFrames[1] <= TargetControlFrames[3]+0.1 && abs(VxDrone)<30 && abs(VyDrone)<30){ // Start Mission condition
			//else if (abs(VxDrone)<30 && abs(VyDrone)<30 && drone_info[2]>=mission_pose[2]-0.5 && drone_info[2]<=mission_pose[2]+0.5){

				if (detector_client.call(detector_srv))
					detector_flag = detector_srv.response.output; 
				else
					std::cout <<"Failed to call service detect_object"<<std::endl;

				if (detector_flag == 0 || detector_flag == -1){ // No object detected
					current_state = FOLLOWING_LEADER;
					std::cout <<"No object detected"<<std::endl;}
				else{ // Something has been detected
					current_state = HOVERING;
					std::cout <<"An Object has been detected"<<std::endl;}

				drone_cam_client.call(drone_cam_srv);
				}
			else if (find_gesture(0,20,LAND_GESTURE)>=90){
				current_state = LANDED;
				land.publish(EmptyMsg);
				}
			else
				current_state = MISSION;
			/*
			std::cout <<" X conditions " <<std::endl;
			std::cout <<TargetControlFrames[0] << " >= " <<  TargetControlFrames[2]-0.05 <<std::endl;
			std::cout <<TargetControlFrames[0] << " <= " <<  TargetControlFrames[2]+0.05 <<std::endl;
			std::cout <<" Y conditions " <<std::endl;
			std::cout <<TargetControlFrames[1] << " >= " <<  TargetControlFrames[3]-0.15 <<std::endl;
			std::cout <<TargetControlFrames[1] << " <= " <<  TargetControlFrames[3]+0.15 <<std::endl;
			std::cout <<" Velocity conditions " <<std::endl;
			std::cout <<"VX   "<<abs(VxDrone)<< " VY   " <<abs(VyDrone)<<std::endl;*/

			break;
		case EMERGENCY: 
			if (find_gesture(0,20,LAND_GESTURE)>=90)
				current_state = LANDED;
			else
				current_state = EMERGENCY;
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
    	gestures[FOLLOW_LEADER_1_GESTURE]="follow_leader_1";
	gestures[FOLLOW_LEADER_2_GESTURE]="follow_leader_2";
	gestures[MISSION_GESTURE]="mission_gesture";

std::map<drone_state,std::string> status;
	// Map definition 
	status[LANDED]="landed";
	status[HOVERING] = "hovering";
	status[FOLLOWING_LEADER]="following_leader";
	status[MISSION] = "mission_mode";
	status[EMERGENCY] = "Emergency";

// Variables declaration 
drone_control_msgs::demo_info data_out;
gesturestype gesture_detected = NON_GESTURE;
drone_state current_state = LANDED;
visual_object_detector::DetectObject detector_srv;
ardrone_autonomy::CamSelect drone_cam_srv;


//Physical user data
std::vector<double> heights;
nh_.getParam("/gestures_node/leaders_heights",heights);
nh_.getParam("/gestures_node/leaders_id",leaders_id);
nh_.getParam("/leader_selector/leader_id",leader_id);
nh_.getParam("/gestures_node/mission_target",mission_pose);
nh_.getParam("/drone_control_node/delta_pose",dPose);

// Publishers and subscribers
ros::Subscriber optitrack_leader_sub_=nh_.subscribe("leader_pose_topic", 1, hasReceivedLeaderState);
ros::Subscriber optitrack_marker_leader_1_sub_=nh_.subscribe("marker_leader_1_pose_topic", 1, hasReceivedMarkerLeader1State);
ros::Subscriber optitrack_marker_leader_2_sub_=nh_.subscribe("marker_leader_2_pose_topic", 1, hasReceivedMarkerLeader2State);
ros::Subscriber optitrack_drone_control_sub_=nh_.subscribe("drone_control_topic", 1, hasReceivedDroneControlState);
ros::Subscriber drone_navdata_sub = nh_.subscribe("/ardrone/navdata", 1, hasReceivedNavdataInfo);

ros::Publisher demo_info_pub_=nh_.advertise<drone_control_msgs::demo_info>("demo_info_topic",1);
ros::Publisher takeoff_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
ros::Publisher land_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/land",1);

// Services
ros::ServiceClient detector_client = nh_.serviceClient<visual_object_detector::DetectObject>("/detect_object");
ros::ServiceClient drone_cam_client =  nh_.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/togglecam");

	
	// Main loop
	while (ros::ok()){

	// add the gesture detected to the gestures queue
	gesture_detected = gesture_detector(heights, leader_id);
	gestures_buffer(gesture_detected);
	drone_status(current_state, takeoff_pub_, land_pub_, nh_, detector_client, detector_srv, drone_cam_client, drone_cam_srv);
	
	// Publishing node topic
	data_out.gesture_detected = gestures[gesture_detected];
	data_out.demo_status = status[current_state];
	demo_info_pub_.publish(data_out);

   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    	}

 return 0;
}
