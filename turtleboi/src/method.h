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


  void Send_cmd(geometry_msgs::Twist intructions);

  void Brake();



  void seperateThread();

  void run();



  // Prameters for ros
  ros::NodeHandle nh_;
  ros::Publisher cmd_velocity;//! Visualisation Marker publisher
  ros::Publisher Led1;
  ros::Publisher Led2;
 




  ros::Subscriber sub1_;  // Few subscribers
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Subscriber sub4_;

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