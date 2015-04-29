#include <iostream>
#include <string>
#include <math.h> 
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/LedAnim.h>
#include <drone_control_msgs/demo_info.h>
#include <sound_play/SoundRequest.h>

std::string drone_status, ant_status ="non_gesture_detected";
double DroneBattery;
int flag = 0;

// Define callbacks
void hasReceivedDemoinfo(const drone_control_msgs::demo_info::ConstPtr& msg){
	
	drone_status = msg->demo_status;
	return;
}

void hasReceivedNavdataInfo(const ardrone_autonomy::NavdataConstPtr msg){
	DroneBattery = msg->batteryPercent;
} 

int main(int argc, char** argv){
    
ros::init(argc, argv, "following_leader_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

ros::Subscriber optitrack_demo_node_sub_=nh_.subscribe("demo_node_topic", 1, hasReceivedDemoinfo);
ros::Subscriber alt_sub = nh_.subscribe("/ardrone/navdata", 10, hasReceivedNavdataInfo);
ros::ServiceClient drone_led =  nh_.serviceClient<ardrone_autonomy::LedAnim>("/ardrone/setledanimation");
ros::Publisher sound_pub_=nh_.advertise<sound_play::SoundRequest>("output_sound", 10);


int BatteryFlag = 0, fs=20;
/* Difinition of the led animation parameters */
ardrone_autonomy::LedAnim srv;
srv.request.freq = fs/5;
sound_play::SoundRequest sound_out;

std::string landed_sound, hovering_sound, following_sound,mission_sound,emergency_sound,object_detected,no_object_found;
nh_.getParam("/drone_feedback_signals_node/emergency_sound",emergency_sound);
nh_.getParam("/drone_feedback_signals_node/landed_sound",landed_sound);
nh_.getParam("/drone_feedback_signals_node/hovering_sound",hovering_sound);
nh_.getParam("/drone_feedback_signals_node/following_sound",following_sound);
nh_.getParam("/drone_feedback_signals_node/mission_sound",mission_sound);
nh_.getParam("/drone_feedback_signals_node/object_detected",object_detected);
nh_.getParam("/drone_feedback_signals_node/no_object_found",no_object_found);

	while (ros::ok()){

	/* Led animation to alert about low battery */
	if(DroneBattery <= 20 && DroneBattery > 10 && BatteryFlag == 0){
		srv.request.duration = 0; 
		srv.request.type = 0;  //BLINK_GREEN_RED
		drone_led.call(srv);
		BatteryFlag = 1;
	}
	else if(DroneBattery <= 10 && BatteryFlag == 1){
		srv.request.duration = 0;
		srv.request.type = 2; // BLINK_RED
		drone_led.call(srv);
		BatteryFlag = 0;
	}

	else if(drone_status != ant_status){

		// Led status feedback
		srv.request.duration = 1.5;
		srv.request.type = 4; // SNAKE_GREEN_RED
		drone_led.call(srv);
		/*
		sound_out.sound = -2;
		sound_out.command = 1;


			if (drone_status == "Emergency")
				sound_out.arg = emergency_sound;
			else if (drone_status == "landed")
				sound_out.arg = landed_sound;
			else if(drone_status == "hovering")
				sound_out.arg = hovering_sound;
			else if(drone_status == "following_leader")
				sound_out.arg = following_sound;
			else if(drone_status == "mission_mode")
				sound_out.arg = mission_sound;

		sound_pub_.publish(sound_out);*/
		ant_status = drone_status;

		}
		
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    	}

 return 0;
}
