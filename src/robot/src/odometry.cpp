#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <inttypes.h>

float ENCODER_VEL[8] = {0};
float WHEEL_RADIUS = .345;
float WHEEL_DIAMETER = WHEEL_RADIUS * M_PI * 2.0;
float AXEL_LEN = .625;

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	for (int i = 0; i < 8; ++i)
	{
		ENCODER_VEL[i] = msg->data[i];
	}
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"odometry_publisher");

	ros::NodeHandle n;
	ros::Subscriber encoder_sub = n.subscribe("encoder_speed", 1000, chatterCallback);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
	
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double theta = 0.0;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	double encoder_vel_l = 0.0;
	double encoder_vel_r = 0.0;
	
	tf::TransformBroadcaster broadcaster;

	ros::Rate r(1.0);
	while(n.ok())
	{
		ros::spinOnce();
		current_time = ros::Time::now();
		encoder_vel_l = ENCODER_VEL[0] * 0.00277778;
		encoder_vel_r = ENCODER_VEL[1] * 0.00277778;
		double delta_t = (current_time - last_time).toSec();
		double dist_l = WHEEL_DIAMETER * encoder_vel_l * delta_t;
		double dist_r = WHEEL_DIAMETER * encoder_vel_r * delta_t;
		double delta_x = cos(theta) * (dist_l/2.0 + dist_r/2.0);
		double delta_y = sin(theta) * (dist_l/2.0 + dist_r/2.0);
		double delta_theta = dist_l/AXEL_LEN - dist_r/AXEL_LEN;

		x += delta_x;
		y += delta_y;
		theta += delta_theta;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
		
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = z;
		odom_trans.transform.rotation = odom_quat;

		odom_broadcaster.sendTransform(odom_trans);

		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = (dist_l/2.0 + dist_r/2.0)/delta_t;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = delta_theta/delta_t;

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.48,0.0,0.0)),
				current_time,"base_link", "base_laser"));

		odom_pub.publish(odom);
		last_time = current_time;
		r.sleep();
		
	}
}		
