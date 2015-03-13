#include <iostream>
#include <string>
#include <math.h> 
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/LedAnim.h>
#include <drone_control_msgs/demo_info.h>

std::string drone_status, ant_status ="non_gesture_detected";
double DroneBattery;

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


int BatteryFlag = 0, fs=20;
/* Difinition of the led animation parameters */
ardrone_autonomy::LedAnim srv;
srv.request.freq = fs/5;


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
		srv.request.duration = 3;
		srv.request.type = 4; // SNAKE_GREEN_RED
		drone_led.call(srv);
		ant_status = drone_status;
		}
		
	
		
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    	}

 return 0;
}
