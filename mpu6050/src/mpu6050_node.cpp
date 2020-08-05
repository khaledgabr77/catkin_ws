#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define PI     (3.14159265F)
//Global Variables
 //int16_t mx,my,mz;
 float ax,ay,az;
 //float gx,gy,gz;
 double roll,pitch,yaw;
//i2c file of compass
int mg;

//prototypes
void checkRC(int rc, char *text) ;
void compassInit();
 double read_compass_data();
 //
using namespace std;

const int I2C_ADDR = 0x68;
const int PWR_MGMT_1 = 0x6B;

float read_word_2c(int fd, int addr) {
  int high = wiringPiI2CReadReg8(fd, addr);
  int low = wiringPiI2CReadReg8(fd, addr+1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

int main(int argc, char **argv) {

  // Connect to device.
  compassInit();
 
  int fd = wiringPiI2CSetup(I2C_ADDR);
  if (fd == -1) {
    printf("no i2c device found?\n");
    return -1;
  } 
    

  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(fd, PWR_MGMT_1, 0);
	delay(100);
   wiringPiI2CWriteReg16(fd,0x6B,1);
  // Start ROS node stuff.
  ros::init(argc, argv, "mpu6050");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
  ros::Rate rate(10);  // hz

  // Publish in loop.
  while(ros::ok()) {
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "imu_link";  // imu_link frame
    
    //covariance matricies
    msg.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    msg.angular_velocity_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    msg.linear_acceleration_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
     
  uint8_t ST1=0;
    do{
      ST1=wiringPiI2CReadReg8(mg, 0x02);
     }while(!(ST1&0x01));
 
  int16_t mx=-(int16_t)((wiringPiI2CReadReg8(mg, 0x04) << 8 )|(wiringPiI2CRead(mg)));
  int16_t my =(int16_t) ((wiringPiI2CRead(mg)<<8) | (wiringPiI2CRead(mg)));
  int16_t mz =(int16_t) (( wiringPiI2CRead(mg)<<8)|(wiringPiI2CRead(mg)));
   ROS_INFO(" mx = %d,my=%d,mz=%d",mx,my,mz);

    // At default sensitivity of 250deg/s we need to scale by 131.
    msg.angular_velocity.x = read_word_2c(fd, 0x43) / 131;
    msg.angular_velocity.y = read_word_2c(fd, 0x45) / 131;
    msg.angular_velocity.z = read_word_2c(fd, 0x47) / 131;
    ROS_INFO("gy.X = %f\t ",msg.angular_velocity.x );
    ROS_INFO("gy.y = %f\t ",msg.angular_velocity.y );
    ROS_INFO("gy.z = %f\t ",msg.angular_velocity.z );



    // Read accelerometer values.
    // At default sensitivity of 2g we need to scale by 16384.
    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
    // But! Imu msg docs say acceleration should be in m/2 so need to *9.807
    const float la_rescale = 16384.0 / 9.807;
    ax  = read_word_2c(fd, 0x3b) ;
    ay = read_word_2c(fd, 0x3d) ;
    az  = read_word_2c(fd, 0x3f);
    
    roll = (float)atan2(ay, az);
    msg.orientation.x = roll;

    if ((ay * sin(roll) + az * cos(roll))==0){
      if (ax>0){
        pitch = PI / 2;
      } else{
        pitch = -PI / 2;
      }
    }else{
      pitch = (float)atan(-ax / (ay * sin(roll) + az * cos(roll)));
    }

    msg.orientation.y = pitch;
 
    yaw = (float)atan2(mz * sin(roll) - my * cos(roll), mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll));

    msg.orientation.z = yaw;

    msg.orientation.w = 1;

      
      ROS_INFO("roll= %f,pitch=%f,yaw=%f",roll,pitch,yaw);
  
    // Read gyroscope values.

    
      msg.linear_acceleration.x=ax/la_rescale;
      msg.linear_acceleration.y =ay/la_rescale;
      msg.linear_acceleration.z=az/la_rescale;
      ROS_INFO("accel.X = %f\t ",msg.linear_acceleration.x);
      ROS_INFO("accel.y = %f\t ",msg.linear_acceleration.y);
      ROS_INFO("accel.z = %f\n ",msg.linear_acceleration.z);



    // Pub & sleep.
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
/*
 * Compass initialize
 * */
 void compassInit(){
  printf("IMU Compass  starting\n");

  // Open an I2C connection
   mg = wiringPiI2CSetup(0x0C); //7 bit address +RW_bit
  checkRC(mg, "wiringPiI2CSetup");
 wiringPiI2CWriteReg8(mg,0x37,0x02); 
 wiringPiI2CWriteReg8(mg,0x0A,0x16);
 
 //adjx=wiringPiI2CReadReg8(fd,0x10); //000110+1=0001 1001 =0x19
 //adjy=wiringPiI2CReadReg8(fd,0x11);
}//compass init

void checkRC(int rc, char *text) {
  if (rc < 0) {
    printf("Error: %s - %d\n", text, rc);
    exit(-1);
  }
}
//****************

double read_compass_data(){
  uint8_t ST1;
printf("before do");
 
 do{
   ST1=wiringPiI2CReadReg8(mg, 0x02);
 }while(!(ST1&0x01));
 
  puts("reading"); 
  int16_t x=-(int16_t)((wiringPiI2CReadReg8(mg, 0x04) << 8 )|(wiringPiI2CRead(mg)));
  
  int16_t y =-(int16_t) ((wiringPiI2CRead(mg)<<8) | (wiringPiI2CRead(mg)));
  int16_t  z =-(int16_t) (( wiringPiI2CRead(mg)<<8)|(wiringPiI2CRead(mg)));

    //  ROS_INFO("compass x = %f",x);
      
    //  ROS_INFO("compass y = %f",y);

    //  ROS_INFO("compass z = %f",z);
 //  int16_t x_sens=x*((((adjx-128)*0.5)/128)+1); 
  // int16_t y_sens=y*((((adjy-128)*0.5)/128)+1);
   
   double angle = atan2((double) y, (double)x) * (180 / PI) + 180;
 
   
   return angle;

  }
