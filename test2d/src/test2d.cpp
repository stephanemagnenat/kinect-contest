#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <iostream>
#include <cmath>

using namespace std;

const string baseLinkFrame = "/base_link";
const string odomFrame = "/odom";
const string kinectFrame = "/openni_depth";
const string worldFrame = "/world";
const double limit_low = 1e-3;
const double limit_high = 1e4;

// see http://www.ros.org/wiki/Clock for how to manage timing 
/*
	Execute the following to use this program:
		roscore
		rosparam set /use_sim_time true
		rosbag play ../data/2011-01-13-13-56-40.bag --clock
		bin/test2d
*/

ostream& operator<< (ostream& os, const tf::Quaternion& quat)
{
	os << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w();
	return os;
}

ostream& operator<< (ostream& os, const tf::Vector3& trans)
{
	os << trans.x() << ", " << trans.y() << ", " << trans.z();
	return os;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test2d");

	ros::NodeHandle node;
	
	tf::TransformListener t;
	tf::StampedTransform tr_o, tr_i;
	
	double a_test(0);
	double b_test(0);
	double theta_test(0);
	
	ROS_INFO_STREAM("waiting for initial transforms");
	while (node.ok())
	{
		ros::Time now(ros::Time::now());
		//ROS_INFO_STREAM(now);
		if (t.waitForTransform(baseLinkFrame, now, baseLinkFrame, now, odomFrame, ros::Duration(0.1)))
			break;
		//ROS_INFO("wait");
		//ros::Duration(0.1).sleep();
	}
	ROS_INFO_STREAM("got first odom to baseLink");
	while (node.ok())
	{
		ros::Time now(ros::Time::now());
		//ROS_INFO_STREAM(now);
		if (t.waitForTransform(kinectFrame, now, kinectFrame, now, worldFrame, ros::Duration(0.1)))
			break;
		//ROS_INFO("wait");
		//ros::Duration(0.1).sleep();
	}
	ROS_INFO_STREAM("got first world to kinect");
	
	sleep(3);
	
	ros::Rate rate(0.5);
	while (node.ok())
	{
		// sleep
		rate.sleep();
		
		// get parameters from transforms
		ros::Time curTime(ros::Time::now());
		ros::Time lastTime = curTime - ros::Duration(2);
		ROS_INFO_STREAM("curTime: " << curTime << ", lastTime: " << lastTime);
		t.waitForTransform(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, ros::Duration(3));
		t.waitForTransform(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, ros::Duration(3));
		t.lookupTransform(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, tr_o);
		ROS_INFO_STREAM("odom to baselink: trans: " << tr_o.getOrigin() << ", rot: " << tr_o.getRotation());
		const double alpha_o = tr_o.getOrigin().x();
		const double beta_o = tr_o.getOrigin().y();
		const double theta_o = atan2(tr_o.getRotation().z(), tr_o.getRotation().w());
		t.lookupTransform(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, tr_i);
		ROS_INFO_STREAM("world to kinect: trans: " << tr_i.getOrigin() << ", rot: " << tr_i.getRotation());
		const double alpha_i = tr_i.getOrigin().x();
		const double beta_i = tr_i.getOrigin().y();
		const double theta_i = atan2(tr_i.getRotation().z(), tr_i.getRotation().w());
		lastTime = curTime;
		
		// compute correspondances, check values to prevent numerical instabilities
		const double r_1 = alpha_o * cos(theta_o) + alpha_o + beta_o * sin(theta_o);
		if (abs(r_1) < limit_low)
		{
			ROS_WARN_STREAM("magnitude of r_1 too low: " << r_1);
			continue;
		}
		const double r_2 = beta_o * cos(theta_o) + beta_o - alpha_o * sin(theta_o);
		const double phi = atan2(r_2, r_1);
		const double R_denom = 2 * sin(theta_o);
		if (abs(R_denom) < limit_low)
		{
			ROS_WARN_STREAM("magnitude of R_denom too low: " << R_denom);
			continue;
		}
		const double R = sqrt(r_1*r_1 + r_2*r_2) / R_denom;
		if (abs(R) > limit_high)
		{
			ROS_WARN_STREAM("magnitude of R too high: " << R);
			continue;
		}
		const double C_1 = (beta_i + beta_i * cos(theta_o) + alpha_i * sin(theta_o)) / R_denom;
		if (abs(C_1) < limit_low)
		{
			ROS_WARN_STREAM("magnitude of C_1 too low: " << C_1);
			continue;
		}
		const double C_2 = (alpha_i + alpha_i * cos(theta_o) + beta_i * sin(theta_o)) / R_denom;
		const double xi = atan2(C_2 + b_test, C_1 + a_test);
		
		// compute new values
		theta_test = M_PI - phi - xi;
		a_test = R * sin(theta_test + phi) - C_1;
		b_test = R * cos(theta_test + phi) - C_2;
		
		if (abs(a_test) < limit_low)
			ROS_WARN_STREAM("small a_test: " << a_test);
		if (abs(b_test) < limit_high)
			ROS_WARN_STREAM("small b_test: " << b_test);
		
		// compute transform
		const tf::Vector3 trans_test = tf::Vector3(a_test, b_test, 0);
		const tf::Quaternion quat_test = tf::Quaternion(0, 0, sin(theta_test / 2), cos(theta_test / 2));
		ROS_INFO_STREAM("Estimated transform: trans: " <<  a_test << ", " << b_test << ", rot: " << atan2(quat_test.z(), quat_test.w()));
	}
	
	return 0;
}
