#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <iostream>
#include <cmath>

using namespace std;

const string baseLinkFrame = "/base_link";
const string odomFrame = "/odom";
const string kinectFrame = "/openni_depth";
const string myKinectFrame = "/kinect";
const string worldFrame = "/world";
//const double limit_low = 1e-3;
//const double limit_high = 1e4;
const double max_diff_angle = 0.17; // around 10 deg
const double min_R_rot = 0.5;
const double max_R_trans = 5.0;
const double min_nu = 0.05;

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
	
	tf::TransformListener t(ros::Duration(20));
	tf::StampedTransform tr_o, tr_i;
	
	double a_test(0);
	double b_test(0);
	double theta_test(0);
	double nu_theta(1);
	double nu_trans(1);
	
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
	
	sleep(10);
	
	ros::Rate rate(0.5);
	while (node.ok())
	{
		// sleep
		rate.sleep();
		
		// get parameters from transforms
		ros::Time curTime(ros::Time::now());
		ros::Time lastTime = curTime - ros::Duration(10);
		//ROS_INFO_STREAM("curTime: " << curTime << ", lastTime: " << lastTime);
		t.waitForTransform(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, ros::Duration(3));
		t.waitForTransform(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, ros::Duration(3));
		t.lookupTransform(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, tr_o);
		//ROS_INFO_STREAM("odom to baselink: trans: " << tr_o.getOrigin() << ", rot: " << tr_o.getRotation());
		const double alpha_o = tr_o.getOrigin().x();
		const double beta_o = tr_o.getOrigin().y();
		const double theta_o = 2*atan2(tr_o.getRotation().z(), tr_o.getRotation().w());
		t.lookupTransform(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, tr_i);
		//ROS_INFO_STREAM("world to kinect: trans: " << tr_i.getOrigin() << ", rot: " << tr_i.getRotation());
		const double alpha_i = tr_i.getOrigin().z();
		const double beta_i = -tr_i.getOrigin().x();
		const double theta_i = 2*atan2(-tr_i.getRotation().y(), tr_i.getRotation().w());
		lastTime = curTime;
		
		ROS_WARN_STREAM("Input odom: ("<<alpha_o<<", "<<beta_o<<", "<<theta_o<<"), icp: ("\
				<<alpha_i<<", "<<beta_i<<", "<<theta_i<<")");
		if (abs(theta_i-theta_o) > max_diff_angle)
		{
			ROS_WARN_STREAM("Angle difference too big: " << abs(theta_i-theta_o));
			continue;
		}

		// compute correspondances, check values to prevent numerical instabilities
		const double R_denom = 2 * sin(theta_o);
		/*if (abs(R_denom) < limit_low)
		{
			ROS_WARN_STREAM("magnitude of R_denom too low: " << R_denom);
			continue;
		}*/
		const double kr_1 = (alpha_o * cos(theta_o) + alpha_o + beta_o * sin(theta_o));
		const double kr_2 = (beta_o * cos(theta_o) + beta_o - alpha_o * sin(theta_o));
		const double phi = atan2(kr_2, kr_1);
		const double r_1 = kr_1/R_denom;
		const double r_2 = kr_2/R_denom;
		const double R = sqrt(r_1*r_1 + r_2*r_2);
		const double kC_1 = (beta_i + beta_i * cos(theta_o) + alpha_i * sin(theta_o));
		const double kC_2 = (alpha_i + alpha_i * cos(theta_o) + beta_i * sin(theta_o));
		//const double xi = atan2(kC_2 + R_denom*b_test, kC_1 + R_denom*a_test);
		const double C_1 = kC_1 / R_denom;
		const double C_2 = kC_2 / R_denom;
		double xi(0);
		if (R_denom)
			xi = atan2(C_1 + a_test, C_2 + b_test);
		else
			xi = atan2(kC_1, kC_2);
		
		// compute new values 
		//double tmp_theta = M_PI/2 - phi  - xi;
		double tmp_theta = xi - phi;
		double tmp_a = R * sin(tmp_theta+phi) - C_1;
		double tmp_b = R * cos(tmp_theta+phi) - C_2;
		//tmp_theta = (tmp_theta<-M_PI)?tmp_theta+2*M_PI:((tmp_theta>M_PI)?tmp_theta-2*M_PI:tmp_theta);
		
		//const double V = sqrt((C_1+a_test)*(C_1+a_test)+(C_2+b_test)*(C_2+b_test));
		/*const double err = sqrt((a_test-tmp_a)*(a_test-tmp_a)+(b_test-tmp_b)*(b_test-tmp_b));
		const double err_pred = sqrt(R*R + V*V -2*R*V*cos(tmp_theta+phi-xi));
		if (abs(err-err_pred)>0.00001)
		{
		ROS_WARN_STREAM("Error="<<err<<" Computed="<<err_pred);
		ROS_WARN_STREAM("chosen: ("<<tmp_a<<", "<<tmp_b<<"); rejected: ("
				<<R*sin(tmp_theta+phi+M_PI)-C_1<<", "<<R*cos(tmp_theta+phi+M_PI)-C_2<<")");
		
		ROS_WARN_STREAM("R: "<<R<<", V: "<<V<<", C_1: "<<C_1<<", C_2: "<<C_2\
				<<", phi: "<<phi<<", xi: "<<xi<<", theta: "<<tmp_theta);
		}*/
		if (R>min_R_rot)
		{
			theta_test = atan2((1-nu_theta)*sin(theta_test)+nu_theta*sin(tmp_theta),
					(1-nu_theta)*cos(theta_test)+nu_theta*cos(tmp_theta));
			nu_theta = max(min_nu, 1/(1+1/nu_theta));
		}
		if (R<max_R_trans)
		{
			a_test = (1-nu_trans)*a_test + nu_trans*tmp_a;
			b_test = (1-nu_trans)*b_test + nu_trans*tmp_b;
			nu_trans = max(min_nu, 1/(1+1/nu_trans));
		}
		
		// compute transform
		const tf::Vector3 trans_test = tf::Vector3(a_test, b_test, 0);
		const tf::Quaternion quat_test = tf::Quaternion(0, 0, sin(theta_test/2), cos(theta_test / 2));
		const tf::Quaternion quat_axes = tf::Quaternion(-0.5, 0.5, -0.5, 0.5);
		ROS_INFO_STREAM("Estimated transform: trans: " <<  a_test << ", " <<
				b_test << ", rot: " << 2*atan2(quat_test.z(), quat_test.w()));

		tf::Transform transform;
		transform.setRotation(quat_test*quat_axes);
		transform.setOrigin(trans_test);
	
		static tf::TransformBroadcaster br;
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
baseLinkFrame, myKinectFrame));
	}
	
	return 0;
}
