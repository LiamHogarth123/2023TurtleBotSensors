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

ThreadingMethod::Method(ros::NodeHandle nh) :
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

void ThreadingMethod::seperateThread() {

    Sensorprocessing scanData(Update_Robot_Image_data()); // gets current scan data from sensors
    Movenment GPS(goal, Current_Odom); // sets next goal 

    std::thread myThread(threadForSensor);
    while (true){
        run();
    }
  
  myThread.join();

}


void threadForSensor:: threadForSensor(){
    while(true){
        scanData.Newdata(Update_Robot_Image_data());
        goal_lock.lock();
        goal = scanData.CalculateMidPoint();
        goal_lock.unlock();
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}


void ThreadingMethod::run()  {

    goal_lock.lock();
    GPS.newGoal(goal, Current_Odom);
    goal_locker.unlock();

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


void ThreadingMethod::Send_cmd(geometry_msgs::Twist intructions){
  cmd_velocity.publish(intructions);

}

void ThreadingMethod::Brake(){
  geometry_msgs::Twist intructions;
  intructions.linear.x = 0;
  intructions.linear.y = 0;
  intructions.linear.z = 0;
  intructions.angular.z = 0;
  cmd_velocity.publish(intructions);
}




//callbacks
///////////////////////////////////////////////////////////////////////////////////////////////

void ThreadingMethod::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker);
  Current_Odom = *odomMsg;
}

void  ThreadingMethod::RGBCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (RGB_locker);
  updated_RGB = *Msg;

}

void ThreadingMethod::LidaCallback(const sensor_msgs::LaserScan::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (Lida_locker);
  updated_Lida = *Msg;
}

void ThreadingMethod::ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);
  updated_imageDepth = *Msg;
}


RobotData ThreadingMethod::Update_Robot_Image_data(){
  
  std::unique_lock<std::mutex> lck1 (RGB_locker);
  std::unique_lock<std::mutex> lck2 (Lida_locker);
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);

  Image_data.depthImage = updated_imageDepth;
  Image_data.laserScan = updated_Lida;
  Image_data.rgbImage = updated_RGB;

  return Image_data;
}
