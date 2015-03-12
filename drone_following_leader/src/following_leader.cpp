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
#include <drone_control_msgs/send_control_data.h>
# define PI           3.14159265358979323846


// Some global variables
drone_control_msgs::send_control_data leader_publish_data;
std::vector<double> Leader_info (4,0), virtual_fence (5), ant_pose(4,0), initial_pose(4,0); 
std::vector<float> leader_quaternion (4,0); 
int marker_id = 1.0, leader_id =2.0, ant_leader_id =2.0; 

double quaternion2angles(std::vector<float> &quaternion){
	double roll, pitch, yaw, Dyaw;
	tf::Quaternion quat(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
	tf::Matrix3x3 m(quat);
	m.getRPY(roll, pitch, yaw );
	Dyaw=(double)(yaw*180)/PI;

return Dyaw;
}

// Define callbacks 
void hasReceivedLeaderState(const optitrack_msgs::RigidBodies::ConstPtr& msg){
	
	if (leader_id != ant_leader_id){
		for (int i=0; i<=msg->rigid_bodies.size(); i++){
			if (leader_id == msg->rigid_bodies[i].id)
			marker_id = i;
		}
	}
	
  	Leader_info[0] = msg->rigid_bodies[marker_id].pose.position.x; 
	Leader_info[1] = msg->rigid_bodies[marker_id].pose.position.y;
	Leader_info[2] = msg->rigid_bodies[marker_id].pose.position.z;
	leader_quaternion[0] = msg->rigid_bodies[marker_id].pose.orientation.x;
	leader_quaternion[1] = msg->rigid_bodies[marker_id].pose.orientation.y;
	leader_quaternion[2] = msg->rigid_bodies[marker_id].pose.orientation.z;
	leader_quaternion[3] = msg->rigid_bodies[marker_id].pose.orientation.w;
	Leader_info[3] = quaternion2angles(leader_quaternion);
	ant_leader_id = leader_id;

  return;
} 


void follow_leader(){

	// x position control
	if (Leader_info[0]>virtual_fence[0]-0.1)
		leader_publish_data.position.x = virtual_fence[0]-0.1;
	else if (Leader_info[0]<virtual_fence[1]+0.1)
		leader_publish_data.position.x = virtual_fence[1]+0.1;
	else	
		leader_publish_data.position.x = Leader_info[0];

	// y position control
	if (Leader_info[1]>virtual_fence[2]-0.1)
		leader_publish_data.position.y = virtual_fence[2]-0.1;
	else if (Leader_info[1]<virtual_fence[3]+0.1)
		leader_publish_data.position.y = virtual_fence[3]+0.1;
	else	
		leader_publish_data.position.y = Leader_info[1];
	
	// z control
	if (Leader_info[2]<0.3)
		leader_publish_data.position.z = 0.3;
	else if (Leader_info[2]>virtual_fence[4]-0.1)
		leader_publish_data.position.z = virtual_fence[4]-0.1;
	else	
		leader_publish_data.position.z = Leader_info[2];

	// yaw control
	leader_publish_data.yaw = Leader_info[3];

return;
}	
int main(int argc, char** argv){
    
ros::init(argc, argv, "following_leader_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

ros::Subscriber optitrack_sub_=nh_.subscribe("leader_pose_topic", 1, hasReceivedLeaderState);
ros::Publisher leader_info_pub_=nh_.advertise<drone_control_msgs::send_control_data>("leader_info_topic", 1);

nh_.getParam("/drone_target_points/initial_pose",initial_pose);
ant_pose[0] = initial_pose[0];
ant_pose[1] = initial_pose[1];
ant_pose[2] = initial_pose[2];
ant_pose[3] = initial_pose[3];

	while (ros::ok()){

	nh_.getParam("/drone_control_node/virtual_fence",virtual_fence);
	nh_.getParam("/drone_target_points/leader_id",leader_id);
	
	if (Leader_info[0]){
		follow_leader();
		ant_pose[0]=Leader_info[0];
		ant_pose[1]=Leader_info[1];
		ant_pose[2]=Leader_info[2];
		ant_pose[3]=Leader_info[3];
		}
	else {
		leader_publish_data.position.x=ant_pose[0];
		leader_publish_data.position.y=ant_pose[1];
		leader_publish_data.position.z=ant_pose[2];
		leader_publish_data.yaw=ant_pose[3];
		}

	// Publishing target info
	leader_info_pub_.publish(leader_publish_data);	
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    	}

 return 0;
}
