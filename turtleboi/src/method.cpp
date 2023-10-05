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

#include "sensorprocessing.cpp"
#include "movenment.cpp"

using std::cout;
using std::endl;

Method::Method(ros::NodeHandle nh) :
  nh_(nh)
  // laserProcessingPtr_(nullptr)
{
//Ros construction
//Subscribing to odometry UGV
  sub1_ = nh_.subscribe("/odom", 1000, &Method::odomCallback,this);

  sub2_ = nh_.subscribe("/scan", 10, &Method::LidaCallback,this);

  sub3_ = nh_.subscribe("/camera/rgb/image_raw", 1000, &Method::RGBCallback, this);

  sub4_ = nh_.subscribe("/camera/depth/image_raw", 1000, &Method::ImageDepthCallback, this);

  cmd_velocity = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);

  // Led1 = nh_.advertise<kobuki_msgs::DigitalOutput>("/mobile_base/commands/led1",10);

  // service_ = nh_.advertiseService("/orange/mission", &Sample::request,this);
  
};

void Method::seperateThread() {
  run();

}


void Method::run()  {
  //runs the program
  Sensorprocessing scanData(Update_Robot_Image_data()); // gets current scan data from sensors
  Movenment GPS(goal, Current_Odom); // sets next goal 


  // test loop
  // while(true) {
  //   geometry_msgs::Twist test;
  //   test.linear.x = 2;
  //   test.linear.z = 1;
  //   test.linear.y = 1;
  //   test.angular.z = 0.2;
  //   Send_cmd(test);
  // }


  //Final loop
  while (true){ 
    scanData.Newdata(Update_Robot_Image_data());
    goal = scanData.CalculateMidPoint();
    std::cout << goal.x << std::endl;
    GPS.newGoal(goal, Current_Odom);
    geometry_msgs::Twist traj =GPS.reachGoal();
    std::cout << traj.linear.x << std::endl;
    std::cout << traj.angular.z << std::endl;
    Send_cmd(traj);
    
    // bool Reached_goal = false;
    // while (!Reached_goal){
    //   Reached_goal = GPS.goal_hit(Current_Odom);
    // }
    // Brake();
    
    //could add section to watch odom as gets close and brake
    
  }
}


void Method::Send_cmd(geometry_msgs::Twist intructions){
  cmd_velocity.publish(intructions);

}

void Method::Brake(){
  geometry_msgs::Twist intructions;
  intructions.linear.x = 0;
  intructions.linear.y = 0;
  intructions.linear.z = 0;
  intructions.angular.z = 0;
  cmd_velocity.publish(intructions);
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


RobotData Method::Update_Robot_Image_data(){
  
  std::unique_lock<std::mutex> lck1 (RGB_locker);
  std::unique_lock<std::mutex> lck2 (Lida_locker);
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);

  Image_data.depthImage = updated_imageDepth;
  Image_data.laserScan = updated_Lida;
  Image_data.rgbImage = updated_RGB;

  return Image_data;
}
