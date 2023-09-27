#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>
// #include <kobuki_msgs/DigitalOutput.h>
#include "ros/ros.h"



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

  sub3_ = nh_.subscribe("/camera/rgb/image_raw ", 1000, &Method::RGBCallback, this);

  sub4_ = nh_.subscribe("/camera/rgb/image_raw ", 1000, &Method::ImageDepthCallback, this);



  cmd_velocity = nh_.advertise<geometry_msgs::Twist>("/cmd",10);

  // Led1 = nh_.advertise<kobuki_msgs::DigitalOutput>("/mobile_base/commands/led1",10);

  // service_ = nh_.advertiseService("/orange/mission", &Sample::request,this);

};


void Method::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){

}

void  Method::RGBCallback(const sensor_msgs::Image::ConstPtr& Msg){

}

void Method::LidaCallback(const sensor_msgs::LaserScan::ConstPtr& odomMsg){

}

void Method::ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg){
  
}



void Method::Send_cmd(geometry_msgs::Twist intructions){
    cmd_velocity.publish(intructions);
  }

void Method::seperateThread() {

  // Wait till start input is given
  //check if all is valid
  
  //Start loop
  //while (true)
  //update image/laser data, 
  //Find point
  //Send to caclulation
  //Cacluation returns data
  //send intructions to turtlebot


}