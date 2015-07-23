#include "ros/ros.h"
#include <sound_play/SoundRequest.h>
#include <drone_feedback_signals/demo_sounds.h>

// Global variables
std::string landed_sound, hovering_sound, following_sound,mission_sound,emergency_sound,object_detected,no_object_found;
sound_play::SoundRequest sound_out;
int flag = 0;

// Call backs
bool call_sound(drone_feedback_signals::demo_sounds::Request  &req, drone_feedback_signals::demo_sounds::Response &res){

	res.output = 1;
	flag = 1;

	//std::cout <<"I'm in the service "<<std::endl;
	if (req.sound_name == "Emergency")
		sound_out.arg = emergency_sound;
	else if (req.sound_name == "landed")
		sound_out.arg = landed_sound;
	else if(req.sound_name == "hovering")
		sound_out.arg = hovering_sound;
	else if(req.sound_name == "following_leader")
		sound_out.arg = following_sound;
	else if(req.sound_name == "mission_mode")
		sound_out.arg = mission_sound;
	else if(req.sound_name == "no_object_found")
		sound_out.arg = no_object_found;
	else if(req.sound_name == "object_detected")
		sound_out.arg = object_detected;
	else{
		flag = 0;
		res.output = -1;}
  return true;
}

int main(int argc, char **argv){

ros::init(argc, argv, "demo_sounds_server");
ros::NodeHandle nh_;
ros::NodeHandle nhp_("~");
ros::Rate rate(20.0);

nhp_.getParam("emergency_sound",emergency_sound);
nhp_.getParam("landed_sound",landed_sound);
nhp_.getParam("hovering_sound",hovering_sound);
nhp_.getParam("following_sound",following_sound);
nhp_.getParam("mission_sound",mission_sound);
nhp_.getParam("object_detected",object_detected);
nhp_.getParam("no_object_found",no_object_found);

ros::Publisher sound_pub_=nh_.advertise<sound_play::SoundRequest>("output_sound", 10);
ros::ServiceServer service = nh_.advertiseService("/demo_sounds", call_sound);
	

	while (ros::ok()){
	/* Publish the sound to reproduce */

	if (flag == 1){
		sound_out.sound = -2;
		sound_out.command = 1;
		sound_pub_.publish(sound_out);
		flag = 0;
		}

	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
	}

 return 0;
}
