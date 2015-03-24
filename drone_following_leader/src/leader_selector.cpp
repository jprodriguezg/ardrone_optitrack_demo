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
std::vector<float> leader_quaternion (4,0); 
int marker_id = 1.0, leader_id =0.0, ant_leader_id =0.0; 

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
	
  	leader_publish_data.position.x = msg->rigid_bodies[marker_id].pose.position.x; 
	leader_publish_data.position.y = msg->rigid_bodies[marker_id].pose.position.y;
	leader_publish_data.position.z = msg->rigid_bodies[marker_id].pose.position.z;
	leader_quaternion[0] = msg->rigid_bodies[marker_id].pose.orientation.x;
	leader_quaternion[1] = msg->rigid_bodies[marker_id].pose.orientation.y;
	leader_quaternion[2] = msg->rigid_bodies[marker_id].pose.orientation.z;
	leader_quaternion[3] = msg->rigid_bodies[marker_id].pose.orientation.w;
	leader_publish_data.yaw = quaternion2angles(leader_quaternion);
	ant_leader_id = leader_id;

  return;
} 


int main(int argc, char** argv){
    
ros::init(argc, argv, "leader_selector_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

ros::Subscriber optitrack_sub_=nh_.subscribe("rigied_bodies_topic", 1, hasReceivedLeaderState);
ros::Publisher leader_info_pub_=nh_.advertise<drone_control_msgs::send_control_data>("leader_info_topic", 1);

	while (ros::ok()){
	nh_.getParam("/leader_selector/leader_id",leader_id);
	leader_info_pub_.publish(leader_publish_data);	
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    	}

 return 0;
}
