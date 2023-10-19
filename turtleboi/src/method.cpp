#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include <image_data_struct.h>
// #include <kobuki_msgs/DigitalOutput.h>
#include "ros/ros.h"

using std::cout;
using std::endl;

Method::Method(ros::NodeHandle nh) :
  nh_(nh)
  // laserProcessingPtr_(nullptr)
{
//Ros construction
//Subscribing to TurtleBot3 ROS features



// Robot 1 -----------------------------------------------------
  sub1_ = nh_.subscribe("tb3_0/odom", 1000, &Method::odomCallback,this);

  sub2_ = nh_.subscribe("tb3_0/scan", 10, &Method::LidaCallback,this);

  sub3_ = nh_.subscribe("tb3_0/camera/rgb/image_raw", 1000, &Method::RGBCallback, this);

  sub4_ = nh_.subscribe("tb3_0/camera/depth/image_raw", 1000, &Method::ImageDepthCallback, this);

  cmd_velocity_tb1 = nh_.advertise<geometry_msgs::Twist>("tb3_0/cmd_vel",10);




  // Robot 2 guider ---------------------

  sub5_ = nh_.subscribe("tb3_1/odom", 1000, &Method::guiderOdomCallback,this);

  cmd_velocity_tb2 = nh.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel",10);


  
}

void Method::seperateThread() {
  while (true){
    singleThread();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}


void Method::singleThread() {

  // scanData.Newdata(Update_Robot_Image_data());
        
  // goal = scanData.findTurtlebot();

  // GPS.newGoal(goal, Current_Odom);

  // geometry_msgs::Twist traj = GPS.reachGoal();

  // std::cout << traj.linear.x << std::endl;
  // std::cout << traj.angular.z << std::endl;
  // Send_cmd_tb1(traj);

  guiderBotMovement();
    
}

void Method::guiderBotMovement(){
    
  // geometry_msgs::Twist test;
  // test.linear.x = 0.1;
  // test.linear.z = 0;
  // test.linear.y = 0;
  // test.angular.z = 0.1;
  // Send_cmd_tb2(test);

  geometry_msgs::Point guiderGoal;
  guiderGoal.x = 20;
  guiderGoal.y = 20;

  GuiderGPS.newGoal(guiderGoal, guider_Odom);
  geometry_msgs::Twist traj = GuiderGPS.reachGoal();
  if (GuiderGPS.goal_hit()) {
    Brake();
    std::cout << "braking" << std::endl;
  }
  else {
    std::cout << traj.linear.x << std::endl;
    std::cout << traj.angular.z << std::endl;
    Send_cmd_tb2(traj);
  }

}



void Method::Send_cmd_tb1(geometry_msgs::Twist intructions){
  cmd_velocity_tb1.publish(intructions);
}

void Method::Send_cmd_tb2(geometry_msgs::Twist intructions){
  cmd_velocity_tb2.publish(intructions);
}

void Method::Brake(){
  geometry_msgs::Twist intructions;
  intructions.linear.x = 0;
  intructions.linear.y = 0;
  intructions.linear.z = 0;
  intructions.angular.z = 0;
  cmd_velocity_tb2.publish(intructions);
}




//callbacks
///////////////////////////////////////////////////////////////////////////////////////////////

void Method::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker);
  Current_Odom = *odomMsg;
}

void  Method::RGBCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (RGB_locker);
  updated_RGB = *Msg;

}

void Method::LidaCallback(const sensor_msgs::LaserScan::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (Lida_locker);
  updated_Lida = *Msg;
}

void Method::ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);
  updated_imageDepth = *Msg;
}

void Method::guiderOdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker2);
  guider_Odom = *odomMsg;
}

RobotData Method::Update_Robot_Image_data(){
  
  std::unique_lock<std::mutex> lck1 (RGB_locker);
  std::unique_lock<std::mutex> lck2 (Lida_locker);
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);

  Image_data.depthImage = updated_imageDepth;
  Image_data.laserScan = updated_Lida;
  Image_data.rgbImage = updated_RGB;

  return Image_data;
}