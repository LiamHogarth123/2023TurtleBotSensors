/**
 @file sample.h
 @brief Header file for the Sample class.
*/

#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <image_data_struct.h>
//Keep only the headers needed




/**
 @class Sample
 @brief A class representing a sample.
*/
class Method
{
public:
  /*
  @brief Default constructor.
  @param nh The ROS node handle.
  */
  Method(ros::NodeHandle nh);


  void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
  void LidaCallback(const sensor_msgs::LaserScan::ConstPtr& Msg);
  void RGBCallback(const sensor_msgs::Image::ConstPtr& Msg);
  void ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg);


  RobotData Update_Robot_Image_data();


  void Send_cmd_tb1(geometry_msgs::Twist intructions);

  // void Send_cmd_tb2(geometry_msgs::Twist intructions);


  void Brake();

  void seperateThread();

  void run();



  // Prameters for ROS
  ros::NodeHandle nh_;
  ros::Publisher cmd_velocity_tb1;
  ros::Publisher cmd_velocity_tb2;
  ros::Publisher Led1;
  ros::Publisher Led2;
 

// Subscribers for turtlebot 1 (tb3_0)
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Subscriber sub4_;

// Subscribers for turtlebot 2 (tb3_1)
  ros::Subscriber sub5_;
  ros::Subscriber sub6_;
  ros::Subscriber sub7_;
  ros::Subscriber sub8_;


  std::mutex odom_locker;
  std::mutex RGB_locker;
  std::mutex Lida_locker;
  std::mutex ImageDepth_locker;

  nav_msgs::Odometry Current_Odom;
  sensor_msgs::Image updated_RGB;
  sensor_msgs::LaserScan updated_Lida;
  sensor_msgs::Image updated_imageDepth;

  RobotData Image_data;
  geometry_msgs::Point goal;
  geometry_msgs::Twist traj;

};

  



#endif // SAMPLE_H