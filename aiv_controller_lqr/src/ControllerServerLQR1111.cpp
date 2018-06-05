#include <ros/ros.h>
#include <aiv_controller_lqr/lqrAction.h>
#include <actionlib/server/simple_action_server.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <aiv_controller_lqr/controllerServerConfig.h>
#include <iostream>



#define PI 3.14159265


float r = 0.077;
float lx = 0.188;
float ly = 0.135;

using namespace std;
float x_dot_enc;
float y_dot_enc ;
float th_dot_enc;

float x_dot_goal;
float y_dot_goal ;
float th_dot_goal ;

float x_dot_d ;
float y_dot_d ;
float th_dot_d ;

//Class for containing the server
class ControllerServer{
public:

	ControllerServer(std::string name):
	as(n, "lqr_control", boost::bind(&ControllerServer::executeCB, this, _1), false),
	action_name(name)
	{
		as.registerPreemptCallback(boost::bind(&ControllerServer::preemptCB, this));

		//Start the server
		as.start();

		//Subscriber current velocity of motors
	//	positionservosub = n2.subscribe("/right_wheel1_velocity", 1, &ControllerServerLQR::SensorCallBackR1, this);
		right_wheel1_enc_sub = n2.subscribe("/right_wheel1_velocity", 1, &ControllerServer::SensorCallBackR1, this);
		left_wheel2_enc_sub = n2.subscribe("/left_wheel2_velocity", 1, &ControllerServer::SensorCallBackL2, this);
		left_wheel3_enc_sub = n2.subscribe("/left_wheel3_velocity", 1, &ControllerServer::SensorCallBackL3, this);
		right_wheel4_enc_sub = n2.subscribe("/right_wheel4_velocity", 1, &ControllerServer::SensorCallBackR4, this);

		//Publisher setpoint, current velocity and error of control
		error_control_xpub = n2.advertise<geometry_msgs::Vector3>("/control/error/x", 1);
		error_control_ypub = n2.advertise<geometry_msgs::Vector3>("/control/error/y", 1);
		error_control_thpub = n2.advertise<geometry_msgs::Vector3>("/control/error/th", 1);


		//Publisher LQR output in velocity
		velocityR1pub = n2.advertise<std_msgs::Float32>("/motor/right1vel", 1);
		velocityL2pub = n2.advertise<std_msgs::Float32>("/motor/left2vel", 1);
		velocityL3pub = n2.advertise<std_msgs::Float32>("/motor/left3vel", 1);
		velocityR4pub = n2.advertise<std_msgs::Float32>("/motor/right4vel", 1);



		//Max e Min Output LQR Controller
		float max = PI;
		float min = -PI;

		//Initializing LQR Controller
		Initialize(min,max);
  }


void preemptCB()
{
	ROS_INFO("%s got preempted!", action_name.c_str());
	result.ok = 0;
	as.setPreempted(result, "I got Preempted!");
}



//Callback for processing a goal
void executeCB(const aiv_controller_lqr::lqrGoalConstPtr& goal)
{
  prevTime = ros::Time::now();

	//If the server has been killed, don't process
	if(!as.isActive()||as.isPreemptRequested()) return;

	//Run the processing at 100Hz
	ros::Rate rate(200);

	//Setup some local variables
	bool success = true;
    
	//Loop control
	while(1)
	{
		std_msgs::Float32 msg_vel1;
		std_msgs::Float32 msg_vel2;
		std_msgs::Float32 msg_vel3;
		std_msgs::Float32 msg_vel4;

		x_dot_enc = (velocity_encoderR1 + velocity_encoderL2 + velocity_encoderL3 + velocity_encoderR4)/4;
		y_dot_enc = (velocity_encoderR1 - velocity_encoderL2 + velocity_encoderL3 - velocity_encoderR4)/4;
		th_dot_enc = (velocity_encoderR1 - velocity_encoderL2 - velocity_encoderL3 + velocity_encoderR4)/4;

		x_dot_d = LQRControllerErr(goal->x_dot,x_dot_enc);
		y_dot_d = LQRControllerErr(goal->y_dot,y_dot_enc);
		th_dot_d = LQRControllerErr(goal->th_dot,th_dot_enc); 


		msg_vel1.data = 10.2397*th_dot_goal - 10.1716*th_dot_d + 30.9839*x_dot_goal + 31.0162*x_dot_d + 30.9839*y_dot_goal + 31.0162*y_dot_d;
		msg_vel2.data = -10.2397*th_dot_goal + 10.1716*th_dot_d + 30.9839*x_dot_goal + 31.0162*x_dot_d - 30.9839*y_dot_goal - 31.0162*y_dot_d;
		msg_vel3.data = -10.2397*th_dot_goal + 10.1716*th_dot_d + 30.9839*x_dot_goal + 31.0162*x_dot_d + 30.9839*y_dot_goal + 31.0162*y_dot_d;
		msg_vel4.data =	-10.2397*th_dot_goal - 10.1716*th_dot_d + 30.9839*x_dot_goal + 31.0162*x_dot_d - 30.9839*y_dot_goal - 31.0162*y_dot_d; 
 /*

		//LQR Controller
		msg_vel1.data = -6.3188*(goal->th_dot-th_dot_enc) - 16.0007*(goal->x_dot-x_dot_enc) - 16.0007*(goal->y_dot-y_dot_enc);
		msg_vel2.data = 6.3188*(goal->th_dot-th_dot_enc)  - 16.0007*(goal->x_dot-x_dot_enc) + 16.0007*(goal->y_dot-y_dot_enc);
		msg_vel3.data = 6.3188*(goal->th_dot-th_dot_enc)  - 16.0007*(goal->x_dot-x_dot_enc) - 16.0007*(goal->y_dot-y_dot_enc);
		msg_vel4.data =	-6.3188*(goal->th_dot-th_dot_enc)  - 16.0007*(goal->x_dot-x_dot_enc) + 16.0007*(goal->y_dot-y_dot_enc); */
/*
		//LQR Controller
		msg_vel1.data = -5.054*goal->th_dot - 15.4976*goal->x_dot - 15.4976*goal->y_dot;
		msg_vel2.data = 5.054*goal->th_dot - 15.4976*goal->x_dot + 15.4976*goal->y_dot;
		msg_vel3.data = 5.054*goal->th_dot - 15.4976*goal->x_dot - 15.4976*goal->y_dot;
		msg_vel4.data =	-5.054*goal->th_dot - 15.4976*goal->x_dot + 15.4976*goal->y_dot;
*/
		//Publishing LQE output in velocity
		velocityR1pub.publish(msg_vel1);
		velocityL2pub.publish(msg_vel2);
		velocityL3pub.publish(msg_vel3);
		velocityR4pub.publish(msg_vel4);

		

		//Auxiliary Message
		geometry_msgs::Vector3 msg_errorR1;
		geometry_msgs::Vector3 msg_errorL2;
		geometry_msgs::Vector3 msg_errorL3;
		geometry_msgs::Vector3 msg_errorR4;

               
		msg_errorR1.x = goal->x_dot;
		msg_errorR1.y = x_dot_enc;
		msg_errorR1.z = goal->x_dot - x_dot_enc;

		msg_errorL2.x = goal->y_dot;
		msg_errorL2.y = y_dot_enc;
		msg_errorL2.z = goal->y_dot - y_dot_enc;

		msg_errorL3.x = goal->th_dot;
		msg_errorL3.y = th_dot_enc;
		msg_errorL3.z = goal->th_dot - th_dot_enc;

		//Publishing setpoint, feedback and error control
		error_control_xpub.publish(msg_errorR1);
		error_control_ypub.publish(msg_errorL2);
		error_control_thpub.publish(msg_errorL3);

	   
		feedback.x_dot = x_dot_enc;
		feedback.y_dot = y_dot_enc;
		feedback.th_dot = th_dot_enc;

        //Publish feedback to action client
    	as.publishFeedback(feedback);

		//Check for ROS kill
		if(!ros::ok())
		{
			success = false;
			ROS_INFO("%s Shutting Down", action_name.c_str());
			break;
		}

		//If the server has been killed/preempted, stop processing
		if(!as.isActive()||as.isPreemptRequested()) return;

		//Sleep for rate time
		rate.sleep();
	}

	//Publish the result if the goal wasn't preempted
	if(success)
	{
		result.ok = 1;
		as.setSucceeded(result);
	}
	else
	{
		result.ok = 0;
		as.setAborted(result,"I Failed!");
	}
}


void Initialize( float min, float max)
{
  setOutputLimits(min, max);
  lastError = 0;
  errSum = 0;
}

void setOutputLimits(float min, float max)
{
	if (min > max) return;

	minLimit = min;
	maxLimit = max;
}

float LQRControllerErr(float setpoint, float PV)
{
	ros::Time now = ros::Time::now();
	ros::Duration change = now - prevTime;

	float error = setpoint - PV;

        if(error < 0.125 && error > -0.125){
                error =0;
        }

	errSum += error*change.toSec();
	
  	
	//Do the full calculation
	float output = errSum;

	//Clamp output to bounds
//	output = std::min(output, maxLimit);
//	output = std::max(output, minLimit);

	//Required values for next round
	lastError = error;

	return output;
}


void SensorCallBackR1(const std_msgs::Float32& msg)
{
	velocity_encoderR1 = msg.data;
	velocity_encoderR1 = velocity_encoderR1  + (1-0.5)*velocity_encoderR1;
}

void SensorCallBackL2(const std_msgs::Float32& msg)
{
	velocity_encoderL2 = msg.data;
	velocity_encoderL2 = velocity_encoderL2  + (1-0.5)*velocity_encoderL2;
}


void SensorCallBackL3(const std_msgs::Float32& msg)
{
	velocity_encoderL3 = msg.data;
	velocity_encoderL3 = velocity_encoderL3  + (1-0.5)*velocity_encoderL3;
}


void SensorCallBackR4(const std_msgs::Float32& msg)
{
	velocity_encoderR4 = msg.data;
	velocity_encoderR4 = velocity_encoderR4  + (1-0.5)*velocity_encoderR4;
}


protected:
	ros::NodeHandle n;
	ros::NodeHandle n2;

	//Subscriber
	ros::Subscriber right_wheel1_enc_sub;
	ros::Subscriber left_wheel2_enc_sub;
	ros::Subscriber left_wheel3_enc_sub;
	ros::Subscriber right_wheel4_enc_sub;

	//Publishers
	ros::Publisher velocityR1pub;
	ros::Publisher velocityL2pub;
	ros::Publisher velocityL3pub;
	ros::Publisher velocityR4pub;

	ros::Publisher error_control_xpub;
   	 ros::Publisher error_control_ypub;
	ros::Publisher error_control_thpub;

	//Actionlib variables
	actionlib::SimpleActionServer<aiv_controller_lqr::lqrAction> as;
	aiv_controller_lqr::lqrFeedback feedback;
	aiv_controller_lqr::lqrResult result;
	std::string action_name;

	//Control variables
	float position_encoder;
	float velocity_encoderR1;
	float velocity_encoderL2;
	float velocity_encoderL3;
	float velocity_encoderR4;
	float errSum;
	float lastError;
	float minLimit, maxLimit;
	ros::Time prevTime;

};




//Used by ROS to actually create the node. Could theoretically spawn more than one server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "lqr_server");

 	//Just a check to make sure the usage was correct
	if(argc != 1)
	{
		ROS_INFO("Usage: lqr_server");
		return 1;
	}

	
	//Spawn the server
	   ControllerServer server(ros::this_node::getName());

	ros::spin();

	return 0;
}
