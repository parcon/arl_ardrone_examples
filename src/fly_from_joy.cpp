/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah


This code actuates the ARdrone from a generic joystick message. It is open loop.
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>

double joy_x_,joy_y_,joy_z_;
double joy_x,joy_y,joy_z;
int new_msg=0;
float forget =0.99;
double joy_x_old,joy_y_old,joy_z_old;
geometry_msgs::Twist twist_msg;
std_msgs::Empty emp_msg;
geometry_msgs::Vector3 v3_msg; //[x, y,z]
sensor_msgs::Joy joy_msg_in;

	
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	//Take in joystick
	joy_x_=joy_msg_in.axes[0];
	joy_y_=joy_msg_in.axes[1];
	joy_z_=joy_msg_in.axes[2];
	
	//Take in time
	//msg_time=(double)ros::Time::now().toNSec();
    new_msg=1;
}
	
float map(float value, float in_min, float in_max, float out_min, float out_max) {
  return (float)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}	

int main(int argc, char** argv)
{
	ros::init(argc, argv,"ARDrone_fly_from_joy");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
	ros::Publisher pub_twist;
	ros::Publisher pub_empty;
	ros::Publisher pub_v3;
	ros::Subscriber joy_sub;

    pub_twist = node.advertise<geometry_msgs::Twist>("cmd_vel", 1); //send robot input on /cmd_vel topic
    pub_v3 = node.advertise<geometry_msgs::Vector3>("joy_vel", 1);  //send velocity for graphing on /joy_vel topic
	joy_sub = node.subscribe("/joy", 1, joy_callback); //suscribe to the joystick message
	
    
    ROS_INFO("Waiting for joystick message");
    while (!new_msg){ 
    ros::spinOnce();
	loop_rate.sleep();
    }
    
    ROS_INFO("Starting Joy --> cmd_vel Node");
 	while (ros::ok() && new_msg) { //start the node when a messge comes in
		//pub_empty.publish(emp_msg); //launches the drone

		joy_x=map(joy_x_,-1024,1024,-1,1); //map the joy input (generally 10 bit to what the AR.Drone drivers expect)
		joy_y=map(joy_y_,-1024,1024,-1,1);
		joy_z=map(joy_z_,-1024,1024,-1,1);

		if (fabs(joy_x)<0.01) {joy_x =0;}
		//else {joy_x=joy_x*forget+joy_x_old*(1-forget);} //This line can smoothing the input signal via the float forget. 

		if (fabs(joy_y)<0.01) {joy_y =0;}
		//else {joy_y=joy_y*forget+joy_y_old*(1-forget);}

		if (fabs(joy_z)<0.01) {joy_z =0;}
		//else {joy_z=joy_z*forget+joy_z_old*(1-forget);} 

		ROS_INFO("new message");

		twist_msg.linear.x=joy_x;
		twist_msg.linear.y=joy_y;	
		twist_msg.linear.z=joy_z;
		twist_msg.angular.z=0.0; //YAW IS DISABLED

		v3_msg.x=joy_x;
		v3_msg.y=joy_y;
		v3_msg.z=joy_z;

		new_msg=0;
		pub_v3.publish(v3_msg); //message is posted for easy graphing
		pub_twist.publish(twist_msg); //send message to the robot

		ros::spinOnce();
		loop_rate.sleep();

		}//ros::ok
ROS_ERROR("ROS::ok failed- Node Closing");
}//main
