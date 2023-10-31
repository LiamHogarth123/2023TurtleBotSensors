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

#include "sensorprocessing.h"
#include "movenment.h"

#include <ros/package.h>




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
  void guiderOdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);

  RobotData Update_Robot_Image_data();





  void Send_cmd_tb1(geometry_msgs::Twist intructions);

  void Send_cmd_tb2(geometry_msgs::Twist intructions);

  void threadForSensor();

  void seperateThread();

  void followingRobotRun();

  bool readGoal(bool house);



  geometry_msgs::Point adjustLaserData(geometry_msgs::Point laser_data, nav_msgs::Odometry Position);

  std::vector<geometry_msgs::Point> adjustLaserDataVector(std::vector<geometry_msgs::Point> laser_data, nav_msgs::Odometry Position);

  geometry_msgs::Point global_To_local(geometry_msgs::Point globalPoint, nav_msgs::Odometry robotPose);


  void guiderBotMovement();

  void singleThread();

  void multiThread();

  void telop();

  void telopDrive(void);
  
  void followingRobotThread();

  geometry_msgs::Point motion_dection(std::vector<geometry_msgs::Point> old_points, std::vector<geometry_msgs::Point> newPoints);

  double distance (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);


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

//mutexs
  std::mutex odom_locker;
  std::mutex odom_locker2;
  std::mutex RGB_locker;
  std::mutex Lida_locker;
  std::mutex ImageDepth_locker;
  std::mutex goal_lock;

//variables for callbacks
  nav_msgs::Odometry Current_Odom;
  nav_msgs::Odometry guider_Odom;
  sensor_msgs::Image updated_RGB;
  sensor_msgs::LaserScan updated_Lida;
  sensor_msgs::Image updated_imageDepth;

//SensorData Object
  RobotData Image_data;

//Geometry variable to do with movenment;
  geometry_msgs::Point goal;
  geometry_msgs::Point goal_gobal_frame;
  geometry_msgs::Twist traj;
  geometry_msgs::Point guiderGoal;

  double goal_index;




//Declaration of class objects
  Movenment GPS;
  Movenment GuiderGPS;
  Sensorprocessing scanData;

  bool Threading_switch;
  std::vector<geometry_msgs::Point> Leader_goals;

  bool debuggingMode;
  bool telop_mode;
  bool house;
  bool default_goals;

  //variables for complex motion dection
  std::vector<geometry_msgs::Point> current_map;

};

  



#endif // SAMPLE_H