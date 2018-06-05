#include <ros/ros.h>
#include <aiv_controller_pid/AIVAction.h>
#include <actionlib/server/simple_action_server.h>

#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <aiv_controller_pid/controllerServerConfig.h>


#define PI 3.14159265
// pid values for x
float x_kp;
float x_ki;
float x_kd;
//pid values for y
float y_kp;
float y_ki;
float y_kd;
//pid values for th
float th_kp;
float th_ki;
float th_kd;

float r = 0.077;
float lx = 0.188;
float ly = 0.135;
float rpm = (1/60)*2*PI;
float x_dot_enc = 0.000;
float y_dot_enc = 0.000;
float th_dot_enc = 0.000;

float x_dot_goal = 0.000;
float y_dot_goal = 0.000;
float th_dot_goal = 0.000;

float x_dot_d = 0.000;
float y_dot_d = 0.000;
float th_dot_d = 0.000;		

//Class for containing the server
class ControllerServer{
public:

	ControllerServer(std::string name):
	as(n, "pid_control", boost::bind(&ControllerServer::executeCB, this, _1), false),
	action_name(name)
	{
		as.registerPreemptCallback(boost::bind(&ControllerServer::preemptCB, this));

		//Start the server
		as.start();

		//Subscriber current velocity of motors
	//	positionservosub = n2.subscribe("/right_wheel1_velocity", 1, &ControllerServer::SensorCallBackR1, this);
		right_wheel1_enc_sub = n2.subscribe("/right_wheel1_velocity", 1, &ControllerServer::SensorCallBackR1, this);
		left_wheel2_enc_sub = n2.subscribe("/left_wheel2_velocity", 1, &ControllerServer::SensorCallBackL2, this);
		left_wheel3_enc_sub = n2.subscribe("/left_wheel3_velocity", 1, &ControllerServer::SensorCallBackL3, this);
		right_wheel4_enc_sub = n2.subscribe("/right_wheel4_velocity", 1, &ControllerServer::SensorCallBackR4, this);

		//Publisher setpoint, current velocity and error of control
		error_control_xpub = n2.advertise<geometry_msgs::Vector3>("/control/error/x", 1);
		error_control_ypub = n2.advertise<geometry_msgs::Vector3>("/control/error/y", 1);
		error_control_thpub = n2.advertise<geometry_msgs::Vector3>("/control/error/th", 1);


		//Publisher PID output in velocity
		pub = n2.advertise<geometry_msgs::Twist>("/cmd_vel/pid", 1);
                

		//Max e Min Output PID Controller
		float max = 0.05;
		float min = -0.05;

		//Initializing PID Controller
		Initialize(min,max);
  }


void preemptCB()
{
	ROS_INFO("%s got preempted!", action_name.c_str());
	result.ok = 0;
	as.setPreempted(result, "I got Preempted!");
}

//Callback for processing a goal
void executeCB(const aiv_controller_pid::AIVGoalConstPtr& goal)
{
  prevTime = ros::Time::now();

	//If the server has been killed, don't process
	if(!as.isActive()||as.isPreemptRequested()) return;

	//Run the processing at 100Hz
	ros::Rate rate(100);

	//Setup some local variables
	bool success = true;


	//Loop controlfloat
	while(1)
	{
		std_msgs::Float32 msg_vel1;
		std_msgs::Float32 msg_vel2;
		std_msgs::Float32 msg_vel3;
		std_msgs::Float32 msg_vel4;
		geometry_msgs::Twist msg;

		x_dot_enc = (velocity_encoderR1 + velocity_encoderL2 + velocity_encoderL3 + velocity_encoderR4)*r/4;
		y_dot_enc = (velocity_encoderR1 - velocity_encoderL2 + velocity_encoderL3 - velocity_encoderR4)*r/4;
		th_dot_enc = (velocity_encoderR1 - velocity_encoderL2 - velocity_encoderL3 + velocity_encoderR4)*r/(4*(lx+ly)); 
	
		
		//PID Controller
		
	//	if(goal->x_dot != 0){
		msg.linear.x =  PIDControllerx(goal->x_dot, x_dot_enc,x_kp,x_ki,x_kd);
	//	}
	//	else{msg.linear.x=0;}
	//	if(goal->y_dot != 0){
		msg.linear.y =  PIDControllery(goal->y_dot, y_dot_enc,y_kp,y_kp,y_kd);
	//	else{msg.linear.y=0;}
	///	if(goal->th_dot != 0){		
		msg.angular.z =  PIDControllerth(goal->th_dot, th_dot_enc,th_kp,th_ki,th_kd);
	///	else{msg.linear.z=0;}

		//Publishing PID output in geometry twist
		
		pub.publish(msg);
		
		
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;
		
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
  errSum1 = 0;
  errSum2 = 0;
   errSum3 = 0;
}

void setOutputLimits(float min, float max)
{
	if (min > max) return;

	minLimit = min;
	maxLimit = max;
}

float PIDControllerx(float setpoint, float PV, float kp, float ki, float kd)
{
	ros::Time now = ros::Time::now();
	ros::Duration change = now - prevTime;

	float error = setpoint - PV;
        
	float dErr = error - lastError;

	errSum1 += error*change.toSec();
	if(setpoint == 0){errSum1 =0;}
	//errSum1 = std::min(errSum1, maxLimit);
	//errSum1 = std::max(errSum1, minLimit);

        dErr = (error - lastError)/change.toSec();
	float output;
	//Do the full calculation
	
	output = (kp*error) + (ki*errSum1) + (kd*dErr);


	//Clamp output to bounds
//	output = std::min(output, setpoint);
//	output = std::max(output, setpoint);

	//Required values for next round
	lastError = error;
	if(setpoint == 0){output = 0;}
	return output;
	


}

float PIDControllery(float setpoint, float PV, float kp, float ki, float kd)
{
	ros::Time now = ros::Time::now();
	ros::Duration change = now - prevTime;

	float error = setpoint - PV;
      
	float dErr = error - lastError;

	errSum2 += error*change.toSec();
		if(setpoint == 0){errSum2 =0;}
	//errSum2 = std::min(errSum2, maxLimit);
	//errSum2 = std::max(errSum2, minLimit);


        dErr = (error - lastError)/change.toSec();
	float output;
	//Do the full calculation
	
	output = (kp*error) + (ki*errSum2) + (kd*dErr);


	//Clamp output to bounds
//	output = std::min(output, setpoint);
//	output = std::max(output, setpoint);

	//Required valuesangular for next round
	lastError = error;
	if(setpoint == 0){output = 0;}
	return output;

	

	
}

float PIDControllerth(float setpoint, float PV, float kp, float ki, float kd)
{
	ros::Time now = ros::Time::now();
	ros::Duration change = now - prevTime;

	float error = setpoint - PV;
       
	float dErr = error - lastError;

	errSum3 += error*change.toSec();
		if(setpoint == 0){errSum3 =0;}
	//errSum3 = std::min(errSum3, maxLimit);
	//errSum3 = std::max(errSum3, minLimit);


        dErr = (error - lastError)/change.toSec();
	float output;
	//Do the full calculation
	
	output = (kp*error) + (ki*errSum3) + (kd*dErr);


	//Clamp output to bounds
	output = std::min(output, setpoint);
	output = std::max(output, setpoint);

	//Required values for next round
	lastError = error;
	if(setpoint == 0){output = 0;}
	return output;


	

	
}




void SensorCallBackR1(const std_msgs::Float32& msg)
{
	velocity_encoderR1 = msg.data;
	//velocity_encoderR1 =  velocity_encoderR1 + velocity_encoderR1*(1-0.9);
}

void SensorCallBackL2(const std_msgs::Float32& msg)
{
	velocity_encoderL2 = msg.data;
	//velocity_encoderL2 =  velocity_encoderL2 + velocity_encoderL2*(1-0.9);
}

void SensorCallBackL3(const std_msgs::Float32& msg)
{
	velocity_encoderL3 = msg.data;
	//velocity_encoderL3 =  velocity_encoderL3 + velocity_encoderL3*(1-0.9);
}

void SensorCallBackR4(const std_msgs::Float32& msg)
{
	velocity_encoderR4 = msg.data;
	//velocity_encoderR4 =  velocity_encoderR4 +velocity_encoderR4*(1-0.9);
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
	ros::Publisher pub;

	ros::Publisher error_control_xpub;
        ros::Publisher error_control_ypub;
	ros::Publisher error_control_thpub;


	//Actionlib variables
	actionlib::SimpleActionServer<aiv_controller_pid::AIVAction> as;
	aiv_controller_pid::AIVFeedback feedback;
	aiv_controller_pid::AIVResult result;
	std::string action_name;

	//Control variables
	float position_encoder;
	float velocity_encoderR1;
	float velocity_encoderL2;
	float velocity_encoderL3;
	float velocity_encoderR4;
	float errSum1;
	float errSum2;
	float errSum3;
	float lastError;
	float minLimit, maxLimit;
	ros::Time prevTime;

};

void callback(aiv_controller_pid::controllerServerConfig &config,float level){
  ROS_INFO("New Values x: [%f] = [%f] = [%f]", config.x_kp, config.x_ki, config.x_kd);
	ROS_INFO("New Values y: [%f] = [%f] = [%f]", config.y_kp, config.y_ki, config.y_kd);

	// load x pid Values
	x_kp = config.x_kp;
	x_ki = config.x_ki;
	x_kd = config.x_kd;
	// load y pid Values
	y_kp = config.y_kp;
	y_ki = config.y_ki;
	y_kd = config.y_kd;
	// load th pid Values
	th_kp = config.th_kp;
	th_ki = config.th_ki;
	th_kd = config.th_kd;


}



//Used by ROS to actually create the node. Could theoretically spawn more than one server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pid_server");

 	//Just a check to make sure the usage was correct
	if(argc != 1)
	{
		ROS_INFO("Usage: pid_server");
		return 1;
	}

	dynamic_reconfigure::Server<aiv_controller_pid::controllerServerConfig> server1;
        dynamic_reconfigure::Server<aiv_controller_pid::controllerServerConfig>::CallbackType f;
        f = boost::bind(&callback, _1, _2);
        server1.setCallback(f);
	//Spawn the server
	   ControllerServer server(ros::this_node::getName());

	ros::spin();

	return 0;
}
