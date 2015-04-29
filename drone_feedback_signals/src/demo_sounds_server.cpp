#include "ros/ros.h"
#include <sound_play/SoundRequest.h>
#include <drone_feedback_signals/demo_sounds.h>

// Global variables
std::string landed_sound, hovering_sound, following_sound,mission_sound,emergency_sound,object_detected,no_object_found;
sound_play::SoundRequest sound_out;

// Call backs
bool call_sound(drone_feedback_signals::demo_sounds::Request  &req, drone_feedback_signals::demo_sounds::Response &res){

	std::cout <<"I'm in the service "<<std::endl;
	if (req.sound_name == "Emergency")
		sound_out.arg = emergency_sound;
	else if (req.sound_name == "landed"){
		res.output = 1;
		sound_out.arg = landed_sound;
		std::cout <<" Calling landing ! "<<std::endl;
		}
	else if(req.sound_name == "hovering")
		sound_out.arg = hovering_sound;
	else if(req.sound_name == "following_leader")
		sound_out.arg = following_sound;
	else if(req.sound_name == "mission_mode")
		sound_out.arg = mission_sound;

	sound_out.arg = emergency_sound;

  return true;
}

int main(int argc, char **argv){



ros::init(argc, argv, "demo_sounds_server");
ros::NodeHandle nh_;


nh_.getParam("/drone_feedback_signals_node/emergency_sound",emergency_sound);
nh_.getParam("/drone_feedback_signals_node/landed_sound",landed_sound);
nh_.getParam("/drone_feedback_signals_node/hovering_sound",hovering_sound);
nh_.getParam("/drone_feedback_signals_node/following_sound",following_sound);
nh_.getParam("/drone_feedback_signals_node/mission_sound",mission_sound);
nh_.getParam("/drone_feedback_signals_node/object_detected",object_detected);
nh_.getParam("/drone_feedback_signals_node/no_object_found",no_object_found);

ros::Publisher sound_pub_=nh_.advertise<sound_play::SoundRequest>("/robotsound", 10);
ros::ServiceServer service = nh_.advertiseService("/demo_sounds", call_sound);
	
	/* Publish the sound to reproduce */
	sound_out.sound = -2;
	sound_out.command = 1;
	sound_pub_.publish(sound_out);
	std::cout <<"Su madre "<<std::endl;

ros::spin();

 return 0;
}
