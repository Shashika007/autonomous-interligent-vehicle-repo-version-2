#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <math.h>

double width_robot = 0.1;
double wheel_radius = 0.077;

float lx = 0.188;
float ly = 0.135;
float r = 0.077;

double vx = 0;
double vy = 0;
double vth = 0;
double R = 0;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	geometry_msgs::Twist twist = twist_aux;
        float vel_x = twist_aux.linear.x;
        float vel_y = twist_aux.linear.y;
        float vel_th = twist_aux.angular.z;
        float right_wheel1_data = 0.000;
        float left_wheel2_data = 0.000;
        float left_wheel3_data = 0.000;
        float right_wheel4_data = 0.000;

        float gain = 1/r;
        right_wheel1_data = gain*(twist_aux.linear.x - twist_aux.linear.y + twist_aux.angular.z*(lx+ly));
        left_wheel2_data = gain*(twist_aux.linear.x + twist_aux.linear.y - twist_aux.angular.z*(lx+ly));
        left_wheel3_data = gain*(twist_aux.linear.x - twist_aux.linear.y - twist_aux.angular.z*(lx+ly));
        right_wheel4_data = gain*(twist_aux.linear.x + twist_aux.linear.y + twist_aux.angular.z*(lx+ly));


//      left_vel = (twist_aux.linear.x - width_robot*twist_aux.angular.z)/wheel_radius;
//      right_vel = (twist_aux.linear.x + width_robot*twist_aux.angular.z)/wheel_radius;
//        vx = (right_wheel1_data + left_wheel2_data + left_wheel3_data + right_wheel4_data)/4 ;
//        vy = (-right_wheel1_data + left_wheel2_data + left_wheel3_data - right_wheel4_data)/4;
        vx = twist_aux.linear.x;
        vy = twist_aux.linear.y;
        vth = twist_aux.angular.z ;
        R =sqrt(pow(vx,2)+pow(vy,2));

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "odom");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_noicy", 50);
	ros::Subscriber cmd_vel_sub = n.subscribe("sensor_velocity", 10, cmd_velCallback);

	// initial position
	double x = 0.0; 
	double y = 0.0;
	double th = 0;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	const double degree = M_PI/180;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	while (ros::ok()) {
		current_time = ros::Time::now(); 

		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th)- vy * sin(th)) * dt;
		double delta_y = (vx * sin(th)+ vy * cos(th)) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		odom_trans.header.stamp = current_time; 
		odom_trans.transform.translation.x = x; 
		odom_trans.transform.translation.y = y; 
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.pose.covariance[0] = (1e-3);
		odom.pose.covariance[7] = (1e-3);
		odom.pose.covariance[14] = (1e-6);
		odom.pose.covariance[21] = (1e-6);
		odom.pose.covariance[28] = (1e-6);
  		odom.pose.covariance[35] = (1e-3);
		//velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vth;
		odom.twist.covariance[0] = 1e-3;
		odom.twist.covariance[7] = 1e-3;
		odom.twist.covariance[14] = 1e-3;
		odom.twist.covariance[21] = 1e-3;
		odom.twist.covariance[35] = 1e-3;
		last_time = current_time;

		// publishing the odometry and the new tf
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
