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

#include <fstream>



using std::cout;
using std::endl;

Method::Method(ros::NodeHandle nh) :
  nh_(nh)
  // laserProcessingPtr_(nullptr)
{
//Ros construction
//Subscribing to TurtleBot3 ROS features

 Threading_switch = false;
 debuggingMode = false;
 telop_mode = false;



 GPS.change_stopping_distance(0.5);
 GuiderGPS.change_stopping_distance(0.1);
 goal_index = 0;

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
  //User input
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::string userInput;
  std::cout << "Please enter control method"<< std::endl;
  std::cout << "1. use provide goals"<< std::endl;
  std::cout << "2. provide a goal"<< std::endl;
  std::cout << "3. control the guider robot with user input"<< std::endl;
  std::cin >> userInput;
  int input_int = std::stoi(userInput);

  switch (input_int) {
    case 1: {

      std::cout << "Please input environment type (h = house, e = empty)"<< std::endl;
      std::cin >> userInput;
      if (userInput == "h"){
        default_goals = true;
        }
      break;
      }
    case 2:{
      Leader_goals.clear();
      geometry_msgs::Point temp;
      std::cout << "enter x"<< std::endl;
      std::cin >> userInput;
      temp.x = std::stoi(userInput);
      std::cout << "enter y"<< std::endl;
      std::cin >> userInput;
      temp.y = std::stoi(userInput);
      Leader_goals.push_back(temp);
      break;
      }
    case 3:{
      telop_mode = true;
      break;
      }
    case 4:{
      std::cout <<"Super Secret debugging bog" << std::endl;
      scanData.Newdata(Update_Robot_Image_data());
      scanData.PrintLaserSpec();
      break;
    }
    default:{
    std::cout << "invalid input - therefore using default goals"<< std::endl;
    }
  }

  // std::cout << "Would you like to use multiThreading for each turtlebot: (y/n)" << std::endl;

  // std::cin >> userInput;

  // if (userInput == "y"){
  //   Threading_switch = true;
  // }



  std::cout << "Please input environment type (h = house, e = empty)"<< std::endl;
  std::cin >> userInput;
  if (userInput == "h"){
    house = true;
    if (default_goals){
      readGoal(true);
    }
  }
  else {
    house = false;
    if (default_goals){
      readGoal(false);
    }
  }





  //Code start
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  if (Threading_switch){
    multiThread();
  }
  else if(telop_mode){
    while (true){
      followingRobotRun();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

  }
  else{
    while (true){
      singleThread();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
}


//threading switching
/////////////////////////////////////////////////////////////////////////////////////////////

void Method::singleThread() {
  followingRobotRun();
  guiderBotMovement();
}

void Method::multiThread(){  
  std::thread Lead_robot_thread(&Method::guiderBotMovement, this);
  while (true){
    followingRobotRun();
  }
}

void Method::followingRobotThread(){
  while(true){
    followingRobotRun();
  }
}


//Movenment control for both 
///////////////////////////////////////////////////////////////////////////////////////////
void Method::guiderBotMovement(){
  
  if (Threading_switch){
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (int i = 0; i < Leader_goals.size(); i++){
      
      guiderGoal = Leader_goals.at(i);

      GuiderGPS.newGoal(guiderGoal, guider_Odom);
      geometry_msgs::Twist traj = GuiderGPS.reachGoal();
      Send_cmd_tb2(traj);
    }
  }
  else{

    // std::cout<< "guiderbotopening" << std::endl;
    geometry_msgs::Point guiderGoal;

    guiderGoal = Leader_goals.at(goal_index);
   

    GuiderGPS.newGoal(guiderGoal, guider_Odom);
    geometry_msgs::Twist guiderTraj = GuiderGPS.guiderReachGoal();
    Send_cmd_tb2(guiderTraj);
    if (GuiderGPS.goal_hit(guiderGoal, guider_Odom)){
      if (goal_index != Leader_goals.size()){
         goal_index++;
      }
    }
  }
}

void Method::followingRobotRun(){
        

  scanData.Newdata(Update_Robot_Image_data());

  
  if (house){
    std::vector<geometry_msgs::Point> NewPoints;
    NewPoints = scanData.findAllLaserPoints();
    NewPoints = adjustLaserDataVector(NewPoints, Current_Odom);

    goal = motion_dection(current_map, NewPoints);
    goal = global_To_local(goal, Current_Odom);

    current_map = NewPoints;
  }
  else {
    goal = scanData.findTurtlebotworld();
  }

  GPS.newGoal(goal, Current_Odom);  
  traj = GPS.reachGoal();
  Send_cmd_tb1(traj);
}


// Publishing functions 
///////////////////////////////////////////////////////////////////////////////////////////
void Method::Send_cmd_tb1(geometry_msgs::Twist intructions){
  cmd_velocity_tb1.publish(intructions);
}

void Method::Send_cmd_tb2(geometry_msgs::Twist intructions){
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




// Read/Load goals
//////////////////////////////////////////////////////////////////////
bool Method::readGoal(bool house) {
    // Define the filename of the text file you want to read
    // std::string filename = "../data/Goals.TXT";
    std::string filename;
    std::string default_filename;
    std::string path;
    
    if (house){
      path = ros::package::getPath("turtleboi");
      path += "/data/"; //Looking at data subfolder
      default_filename = path + "House.TXT";
      nh_.param<std::string>("goals", filename, default_filename);
    }
    else {
      path = ros::package::getPath("turtleboi");
      path += "/data/"; //Looking at data subfolder
      default_filename = path + "Goals.TXT";
      nh_.param<std::string>("goals", filename, default_filename);
    }


    // Open the file for reading
    std::ifstream file(filename, std::ios::in);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        if (file.fail()) {
            std::cerr << "Error code: " << file.rdstate() << std::endl;
        }
        return false; // Return an error code
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        geometry_msgs::Point point;
        if (lineStream >> point.x >> point.y >> point.z) {
          Leader_goals.push_back(point);
        }
        
        else {
          std::cerr << "Error parsing line: " << line << std::endl;    
        }
    }

    file.close();

    for (const auto& point : Leader_goals) {
        std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    return true; 
}


//data Adjustement function
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Point Method::adjustLaserData(geometry_msgs::Point laser_data, nav_msgs::Odometry Position) {
  geometry_msgs::Point adjustedValues;
  
  // Get the orientation from the odometry message
  geometry_msgs::Quaternion orientation = Position.pose.pose.orientation;
  
  // Convert the quaternion to Euler angles (roll, pitch, yaw)
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  
  // Perform the coordinate transformation
  adjustedValues.x = laser_data.x * cos(yaw) - laser_data.y * sin(yaw);
  adjustedValues.y = laser_data.x * sin(yaw) + laser_data.y * cos(yaw);
  
  // Add the position offset
  adjustedValues.x += Position.pose.pose.position.x;
  adjustedValues.y += Position.pose.pose.position.y;
  
  return adjustedValues;
}


std::vector<geometry_msgs::Point> Method::adjustLaserDataVector(std::vector<geometry_msgs::Point> laser_data, nav_msgs::Odometry Position) {
  std::vector<geometry_msgs::Point> adjustedValues;
  geometry_msgs::Point temp;
  
   // Get the orientation from the odometry message
  geometry_msgs::Quaternion orientation = Position.pose.pose.orientation;
  
  // Convert the quaternion to Euler angles (roll, pitch, yaw)
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  for (int i = 0; i < laser_data.size(); i++){
    // Perform the coordinate transformation
    temp.x = laser_data.at(i).x * cos(yaw) - laser_data.at(i).y * sin(yaw);
    temp.y = laser_data.at(i).x * sin(yaw) + laser_data.at(i).y * cos(yaw);
    // Add the position offset
    temp.x += Position.pose.pose.position.x;
    temp.y += Position.pose.pose.position.y;
    adjustedValues.push_back(temp);
  }
  return adjustedValues;
}


geometry_msgs::Point Method::global_To_local(geometry_msgs::Point globalPoint, nav_msgs::Odometry robotPose) {
    // Extract the robot's orientation (Quaternion) from robotPose
  geometry_msgs::Quaternion orientation = robotPose.pose.pose.orientation;

  // Convert the quaternion to Euler angles (roll, pitch, yaw)
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  // Perform the coordinate transformation to move the point back to the original local frame
  double rotatedX = globalPoint.x * cos(yaw) - globalPoint.y * sin(yaw);
  double rotatedY = globalPoint.x * sin(yaw) + globalPoint.y * cos(yaw);

  // Add the position offset to move the point back to the local frame
  double localX = rotatedX + robotPose.pose.pose.position.x;
  double localY = rotatedY + robotPose.pose.pose.position.y;

  geometry_msgs::Point localPoint;
  localPoint.x = localX;
  localPoint.y = localY;

  return localPoint;
}



geometry_msgs::Point Method::motion_dection(std::vector<geometry_msgs::Point> old_points, std::vector<geometry_msgs::Point> newPoints){
  geometry_msgs::Point Motion_point;
  double movementThreshold = 0.01;
  for (const auto& newPoint : newPoints) {
    bool pointMatched = false;
    for (auto& oldPoint : old_points) {
        if (distance(oldPoint, newPoint) < movementThreshold) {
          pointMatched = true;
          oldPoint = newPoint; // Update old point with the new data
          break;
        }
      }
    if (!pointMatched) {
      old_points.push_back(newPoint); // New point detected
      std::cout << "New point detected: (" << newPoint.x << ", " << newPoint.y << ")\n";
      // Perform actions for new points
    }
  }

    // Identify moving points that have changed their positions
  for (const auto& oldPoint : old_points) {
    for (const auto& newPoint : newPoints) {
      if (distance(oldPoint, newPoint) > movementThreshold) {
        std::cout << "Moving point detected: (" << oldPoint.x << ", " << oldPoint.y << ")\n";
        Motion_point = newPoint;
        break;
      }
    }
  }
  
  return Motion_point;
}


double Method::distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}