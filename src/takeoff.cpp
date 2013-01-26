/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah

This program launches the AR Drone. 
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
std_msgs::Empty emp_msg;	

int main(int argc, char** argv)
{

	ROS_INFO("Flying ARdrone");
	ros::init(argc, argv,"ARDrone_test");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
	ros::Publisher pub_empty;
	pub_empty = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */

 	while (ros::ok()) 
 				{
				double time_start=(double)ros::Time::now().toSec();
				while ((double)ros::Time::now().toSec()< time_start+5.0) /* Send command for five seconds*/
					{ 
					pub_empty.publish(emp_msg); /* launches the drone */
					ros::spinOnce();
					loop_rate.sleep();
					}//time loop
				ROS_INFO("ARdrone launched");
				exit(0);
				}//ros::ok loop

}//main
