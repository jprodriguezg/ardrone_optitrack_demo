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
# define PI           3.14159265358979323846


// Some global variables
std::vector<double> gesture_marker_info (4,0), gesture_signal(2,0); 
std::vector<float> quaternion (4,0); 

double quaternion2angles(std::vector<float> &quaternion){
	double roll, pitch, yaw, Dyaw;
	tf::Quaternion quat(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
	tf::Matrix3x3 m(quat);
	m.getRPY(roll, pitch, yaw );
	Dyaw=(double)(yaw*180)/PI;

return Dyaw;
}


void hasReceivedLeaderState(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
	// Obtaining drone info 
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
		
int main(int argc, char** argv){
    
ros::init(argc, argv, "takeoff_land_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

std_msgs::Empty GestureMsg;

ros::Subscriber optitrack_sub_=nh_.subscribe("gesture_marker_pose_topic", 1, hasReceivedLeaderState);
ros::Publisher takeoff_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
ros::Publisher land_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/land",1);

	while (ros::ok()){
	nh_.getParam("/gestures_node/gestures_def",gesture_signal);

	if(gesture_marker_info[2]<gesture_signal[0]){
		land_pub_.publish(GestureMsg);
		std::cout << "Land gesture detected" << std::endl;
	}

	if(gesture_marker_info[2]>gesture_signal[1]){	
		takeoff_pub_.publish(GestureMsg);
		std::cout << "Takeoff gesture detected" << std::endl;
	}	
	

   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    	}

 return 0;
}
