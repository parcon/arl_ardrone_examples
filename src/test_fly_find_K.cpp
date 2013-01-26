/*
Parker Conroy
ARLab

*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

	geometry_msgs::Twist twist_msg;
	geometry_msgs::Twist twist_msg_hover;
	geometry_msgs::Twist twist_msg_neg;
	geometry_msgs::Twist twist_msg_pshover;
	geometry_msgs::Twist twist_msg_up;
	std_msgs::Empty emp_msg;
	double vx_=0.0;
	double vy_=0.0;
	double vz_=0.0;
	float takeoff_time=8.0;
	float fly_time=3.0;
	float land_time=3.0;
	float kill_time =4.0;	
			
void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	//Take in state of ardrone	
	vx_=msg_in.vx*0.001;
	vy_=msg_in.vy*0.001;	
	vz_=msg_in.vz*0.001;	
	//ROS_INFO("getting sensor reading");	
}

geometry_msgs::Twist test_controller(double vx_des,double vy_des,double vz_des,double K)
{
		geometry_msgs::Twist twist_msg_gen;
	
		twist_msg_gen.linear.x=K*(vx_des-vx_); //{-1 to 1}=K*( m/s - m/s)
		//ROS_INFO("vx des- vx sensor &f",vx_des-vx_);
		twist_msg_gen.linear.y=K*(vy_des-vy_); 
		twist_msg_gen.linear.z=K*(vz_des-vz_);
		twist_msg_gen.angular.x=1.0; 
		twist_msg_gen.angular.y=1.0;
		twist_msg_gen.angular.z=0.0;
		return twist_msg_gen;
}

int main(int argc, char** argv)
{

	printf("Manual Test Node Starting");
	ros::init(argc, argv,"ARDrone_manual_test");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

	ros::Publisher pub_empty_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_takeoff;
	ros::Publisher pub_empty_reset;
	ros::Subscriber nav_sub;
	double start_time;

//hover message
			twist_msg_hover.linear.x=0.0; 
			twist_msg_hover.linear.y=0.0;
			twist_msg_hover.linear.z=0.0;
			twist_msg_hover.angular.x=0.0; 
			twist_msg_hover.angular.y=0.0;
			twist_msg_hover.angular.z=0.0;  
//fly up
			twist_msg_up.linear.x=0.0; 
			twist_msg_up.linear.y=0.0;
			twist_msg_up.linear.z=0.5;
			twist_msg_up.angular.x=0.0; 
			twist_msg_up.angular.y=0.0;
			twist_msg_up.angular.z=0.0;			
//command message
			twist_msg.linear.x=0.0; 
			twist_msg.linear.y=0.0;
			twist_msg.linear.z=0.0;
			twist_msg.angular.x=0.0; 
			twist_msg.angular.y=0.0;
			twist_msg.angular.z=0.0;


	nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);	
  	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); 
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); 
	pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1);
	
	start_time =(double)ros::Time::now().toSec();	
	ROS_INFO("Starting ARdrone_test loop");

	double desired_vx=0.75; // [m/s]
	//double desired_vx=0.0; // [m/s]
	double K = .75; // []

while (ros::ok()) {
		while ((double)ros::Time::now().toSec()< start_time+takeoff_time){ //takeoff
		
			pub_empty_takeoff.publish(emp_msg); //launches the drone
				pub_twist.publish(twist_msg_hover); //drone is flat
			ROS_INFO("Taking off");
			ros::spinOnce();
			loop_rate.sleep();
			}//while takeoff

		while  ((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time){
		
			pub_twist.publish(twist_msg_hover); //drone is flat
		
			ROS_INFO("Landing");
			
					
			if ((double)ros::Time::now().toSec()> takeoff_time+start_time+fly_time+land_time+kill_time){
			pub_empty_land.publish(emp_msg); //lands the drone
				ROS_INFO("Closing Node");
				exit(0); 	}//kill node
			ros::spinOnce();
			loop_rate.sleep();			
}//while land

		while ( (double)ros::Time::now().toSec()> start_time+takeoff_time && (double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time){	
		
			twist_msg=test_controller(desired_vx,0.0,0.0,K);

			if((double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time){
			pub_twist.publish(twist_msg);
			ROS_INFO("Flying +ve");

			}//fly according to desired twist
			
			if((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time){
			
			desired_vx=-desired_vx;
			pub_twist.publish(twist_msg);
			ROS_INFO("Flying -ve");

			}//fly according to desired twist
			
			ros::spinOnce();
			loop_rate.sleep();
			}

	ros::spinOnce();
	loop_rate.sleep();

}//ros::ok

}//main
