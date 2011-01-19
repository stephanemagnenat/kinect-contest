#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <iostream>

using namespace std;

const string baseLinkFrame = "/base_link";
const string odomFrame = "/odom";
const string kinectFrame = "/openni_depth";
const string worldFrame = "/world";
const double limit_low = 1e-3;
const double limit_high = 1e4;

// see http://www.ros.org/wiki/Clock for how to manage timing 
// rosbag play 2011-01-13-13-56-40.bag --clock
// rosparam set /use_sim_time true

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
	ros::Rate pollRate(0.5);
	while (node.ok())
	{
		if (t.waitForTransform(baseLinkFrame, ros::Time::now(), baseLinkFrame, ros::Time::now(), odomFrame, ros::Duration(0.1)))
			break;
		ROS_INFO("wait");
		pollRate.sleep();
	}
	ROS_INFO_STREAM("got odom to baseLink");
	while (node.ok())
		if (t.waitForTransform(kinectFrame, worldFrame, ros::Time::now(), ros::Duration(0.1)))
			break;
	ROS_INFO_STREAM("got world to kinect");
	
	sleep(3);
	
	ros::Rate rate(0.5);
	while (node.ok())
	{
		// sleep
		rate.sleep();
		
		// get parameters from transforms
		ros::Time curTime(ros::Time::now());
		//t.getLatestCommonTime(odomFrame, baseLinkFrame, curTime, 0);
		ros::Time lastTime = curTime - ros::Duration(2);
		ROS_INFO_STREAM("curTime: " << curTime << ", lastTime: " << lastTime);
		//t.waitForTransform(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, ros::Duration(3));
		//t.waitForTransform(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, ros::Duration(3));
		t.lookupTransform(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, tr_o);
		ROS_INFO_STREAM("odom to baselink: trans: " << *tr_o.getOrigin() << ", rot: " << *tr_o.getRotation());
		const double alpha_o = tr_o.getOrigin().x();
		const double beta_o = tr_o.getOrigin().y();
		const double theta_o = atan2(tr_o.getRotation().z(), tr_o.getRotation().w());
		t.lookupTransform(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, tr_i);
		ROS_INFO_STREAM("world to kinect: trans: " << *tr_i.getOrigin() << ", rot: " << *tr_i.getRotation());
		const double alpha_i = tr_i.getOrigin().x();
		const double beta_i = tr_i.getOrigin().y();
		const double theta_i = atan2(tr_i.getRotation().z(), tr_i.getRotation().w());
		lastTime = curTime;
	}
	
	return 0;
}
