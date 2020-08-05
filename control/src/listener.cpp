#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <wiringPi.h>

#define PwmPin 18 // pin12

#define F1 5 //29
#define F2 6 //31
#define B1 13 //33
#define B2 19 //35

 void GoForward(){
digitalWrite(F1,HIGH);
digitalWrite(F2,LOW);
digitalWrite(B1,HIGH);
digitalWrite(B2,LOW);
}

 void GoBackward(){
digitalWrite(F1,LOW);
digitalWrite(F2,HIGH);
digitalWrite(B1,LOW);
digitalWrite(B2,HIGH);
}
 void stop(){
digitalWrite(F1,LOW);
digitalWrite(F2,LOW);
digitalWrite(B1,LOW);
digitalWrite(B2,LOW);
} 


float input_angle=90;

void dir_callback(const geometry_msgs::Twist& twist ) {
ROS_INFO_STREAM( " recieved distance =  "<< twist.linear.x <<"\t"<< "recieved Angle = "<< twist.angular.z<<"\n") ;

input_angle += twist.angular.z*5 ;
//duty cycle mapped from milliseconds to scale 2000 ,equavilent to angle needed
float mapped_angle =((input_angle/180)*100)+100 ; 

pwmWrite(PwmPin , mapped_angle);
ROS_INFO_STREAM( "angle \t"<< input_angle);

if (twist.linear.x == 2) {
GoForward();
delay(50);
stop();
ROS_INFO_STREAM( "going_forward \n");
}
else if (twist.linear.x == -2) {
GoBackward();
delay(50);
stop();
ROS_INFO_STREAM( "going_backward \n");
} 
else {
digitalWrite(F1,LOW);
digitalWrite(F2,LOW);
digitalWrite(B1,LOW);
digitalWrite(B2,LOW);
}


}

int main(int argc , char **argv){
setenv("WIRINGPI_GPIOMEM","1",1);
ros::init(argc ,argv,"listener");
ros::NodeHandle nh ;

//cout<<("Starting servo test\n");
  int rc = wiringPiSetupGpio();

  if (rc != 0) {
    //cout<<("Failed to wiringPiSetupGpio()\n");
    return 0;
  }
// dc motors pins
  pinMode(F1,OUTPUT);
  pinMode(F2,OUTPUT);
  pinMode(B1,OUTPUT);
  pinMode(B2,OUTPUT);

  pinMode(PwmPin, PWM_OUTPUT);
  pwmSetMode(PWM_MODE_MS);
  pwmSetClock(192);
  pwmSetRange(2000);

ros::Subscriber sub = nh.subscribe("car_dir" ,1000, &dir_callback);
ros::spin();

}


