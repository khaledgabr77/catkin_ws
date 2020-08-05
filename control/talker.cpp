#include "ros/ros.h"
#include <geometry_msgs/Twist.h>


 int main(int argc, char **argv)
 {
	setenv("WIRINGPI_GPIOMEM","1",1);
   ros::init(argc, argv, "talker"); 
   ros::NodeHandle n;
   ros::Publisher  pub = n.advertise<geometry_msgs::Twist>("car_dir", 1000);
 //  ros::Rate loop_rate(2);
  ros::Rate rate(2);
     while (ros::ok())
     {
      
geometry_msgs::Twist msg;
msg.linear.x= 3;
msg.angular.z = 30;

    pub.publish(msg);
  
ROS_INFO_STREAM( " sending distance =  "<< msg.linear.x << " sending Angle = "<< msg.angular.z) ;
  
     
 //    ros::spinOnce();
 
   //  loop_rate.sleep();

rate.sleep();

    }
  
  
  }



