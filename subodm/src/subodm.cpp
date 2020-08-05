#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>




void dir_callback(const nav_msgs::Odometry::ConstPtr& msg ) {
ROS_INFO("i heared : %f",msg->pose.pose.position.x ) ;


}

int main(int argc , char **argv){

ros::init(argc ,argv,"subodm");
ros::NodeHandle nh ;


ros::Subscriber sub = nh.subscribe("odometry/filtered_map" ,1000, &dir_callback);
ros::spin();
return 0 ;
}
