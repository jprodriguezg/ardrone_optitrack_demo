#include <iostream>
#include <string>
#include <math.h> 
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <optitrack_msgs/RigidBodyData.h>
#include <optitrack_msgs/RigidBodies.h>
#include <drone_control_msgs/demo_info.h>
#include <drone_control_msgs/send_control_data.h>
# define PI           3.14159265358979323846

// Some global variables
drone_control_msgs::send_control_data leader_publish_data;
std::vector<double> Leader_info (4,0), virtual_fence (5), ant_pose(4,0), initial_pose(4,0), dPose (4,0); 
std::string drone_status;

// Define callbacks 
void hasReceivedLeaderState(const drone_control_msgs::send_control_data::ConstPtr& msg){
	
	// Obtaining leader info 
  	Leader_info[0] = msg->position.x; 
	Leader_info[1] = msg->position.y;
	Leader_info[2] = msg->position.z;
	Leader_info[3] = msg->yaw;
  return;
} 

void hasReceivedDemoinfo(const drone_control_msgs::demo_info::ConstPtr& msg){
	drone_status = msg->demo_status;
	return;
}

// Defining Functions
void follow_leader(){
	// Conditions to avoid that drone goes out of fence
	// x position control 
	if (Leader_info[0]>virtual_fence[0]-dPose[0]-0.2) // -0.1
		leader_publish_data.position.x = virtual_fence[0]-dPose[0]-0.2; // -0.1
	else if (Leader_info[0]<virtual_fence[1]+dPose[0]+0.2) // +0.1
		leader_publish_data.position.x = virtual_fence[1]+dPose[0]+0.2; // +0.1
	else	
		leader_publish_data.position.x = Leader_info[0];

	// y position control
	if (Leader_info[1]>virtual_fence[2]-dPose[1]-0.2) // -0.1
		leader_publish_data.position.y = virtual_fence[2]-dPose[1]-0.2; // -0.1
	else if (Leader_info[1]<virtual_fence[3]+dPose[1]+0.2) // +0.1
		leader_publish_data.position.y = virtual_fence[3]+dPose[1]+0.2; // +0.1
	else	
		leader_publish_data.position.y = Leader_info[1];
	
	// z control
	if (Leader_info[2]<0.3)
		leader_publish_data.position.z = 0.3;
	else if (Leader_info[2]>virtual_fence[4]-dPose[2]-0.1) // -0.1
		leader_publish_data.position.z = virtual_fence[4]-0.05; //-0.1
	else	
		leader_publish_data.position.z = Leader_info[2];

	// yaw control
	leader_publish_data.yaw = Leader_info[3];

return;
}	
int main(int argc, char** argv){
    
ros::init(argc, argv, "leader_desired_position_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

ros::Subscriber leader_info_sub_=nh_.subscribe("leader_info_topic", 1, hasReceivedLeaderState);
ros::Subscriber optitrack_demo_node_sub_=nh_.subscribe("demo_node_topic", 1, hasReceivedDemoinfo);
ros::Publisher leader_info_pub_=nh_.advertise<drone_control_msgs::send_control_data>("desired_leader_position_topic", 1);

nh_.getParam("/drone_target_points/initial_pose",initial_pose);
ant_pose = initial_pose;

	while (ros::ok()){

	nh_.getParam("/drone_control_node/virtual_fence",virtual_fence);
	nh_.getParam("/drone_control_node/delta_pose", dPose);
	
	if (Leader_info[0] && drone_status =="following_leader"){
		follow_leader();
		ant_pose =Leader_info;
		}
	else {
		leader_publish_data.position.x=ant_pose[0];
		leader_publish_data.position.y=ant_pose[1];
		leader_publish_data.position.z = 1.0;
		leader_publish_data.yaw=ant_pose[3];
		}

	// Publishing target info
	leader_info_pub_.publish(leader_publish_data);	
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    	}

 return 0;
}
