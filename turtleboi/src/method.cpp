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

/**
 * @brief Control the robots based on user input and threading preferences.
 *
 * This function allows the user to specify the control method for the robots and whether multi-threading
 * should be used. It provides options for reading goals, providing a single goal, or manually controlling
 * the guider robot. Based on the user's input, it configures the control method and threading accordingly.

 * The function starts by presenting the user with a menu to select the desired control method. The available
 * options are:
 * 1. Use provided goals (readGoal function).
 * 2. Provide a single goal.
 * 3. Control the guider robot with user input (teleoperation mode).

 * Depending on the selected option, the function configures the control method accordingly. If multi-threading
 * is chosen, it runs 'multiThread' for concurrent control. In teleoperation mode, it spawns a thread to run
 * 'followingRobotThread' and a separate thread for teleoperation. In other cases, it runs 'singleThread' to
 * control the robots sequentially. User preferences for multi-threading are considered when setting 'Threading_switch'.

 * @note This function serves as the entry point for controlling the robots, allowing users to specify the
 * control method and threading settings.
 */
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
      readGoal();
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
    default:{
    std::cout << "invalid input - therefore using default goals"<< std::endl;
    }
  }

  std::cout << "Would you like to use multiThreading for each turtlebot: (y/n)" << std::endl;

  std::cin >> userInput;

  if (userInput == "y"){
    Threading_switch = true;
  }

  
  if (Threading_switch){
    multiThread();
  }
  else if(telop_mode){
    std::thread Lead_robot_thread(&Method::followingRobotThread, this);
    // std::thread guidingRobotDrive(&Method::telopDrive, this);
    telop();


  }
  else{
    while (true){
      singleThread();
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }
}


/**
 * @brief Execute both following and guider robot control sequentially in a single thread.
 *
 * This function sequentially executes both the following robot control and the guider robot control
 * within the same thread. It allows you to manage the behavior of both robots in a single-threaded fashion.

 * The function starts by calling 'followingRobotRun' to control the behavior of the following robot, and
 * then it proceeds to call 'guiderBotMovement' to manage the guider robot's movement.

 * This single-threaded approach is suitable when you do not require concurrent control of the robots and
 * wish to execute their behaviors sequentially in a coordinated manner.

 * @note This function is used for sequential execution of control logic for both the following and guider robots,
 * one after the other, within a single thread.
 */
void Method::singleThread() {
  followingRobotRun();
  guiderBotMovement();
}

/**
 * @brief Start multi-threaded control for the guider and following robots.
 *
 * This function initiates multi-threaded control for the guider and following robots. It creates a new thread
 * to run the 'guiderBotMovement' function, which controls the guider robot's movement. Meanwhile, in the main
 * thread, it continuously calls the 'followingRobotRun' function to control the behavior of the following robot.

 * The function begins by creating a new thread named 'Lead_robot_thread' to execute the 'guiderBotMovement'
 * function, allowing simultaneous control of the guider robot. In the main thread, it enters an infinite loop,
 * continuously executing the 'followingRobotRun' function, which manages the behavior of the following robot.

 * This multi-threaded approach enables concurrent control of both robots, ensuring that the guider and following
 * robots can operate independently in parallel.

 * @note This function sets up a multi-threaded environment for controlling the guider and following robots,
 * enabling concurrent execution of their respective behaviors.
 */
void Method::multiThread(){  
  std::thread Lead_robot_thread(&Method::guiderBotMovement, this);
  while (true){
    followingRobotRun();
  }
}

/**
 * @brief Continuously execute the following robot control thread.
 *
 * This function continuously executes the following robot control thread by repeatedly calling
 * the 'followingRobotRun' function in an infinite loop. It ensures that the following robot remains
 * responsive and actively follows the leader or performs the designated tasks.

 * The function enters a never-ending loop where 'followingRobotRun' is repeatedly invoked. This is a
 * typical pattern for controlling a thread dedicated to a specific task, ensuring that the thread runs
 * continuously and stays responsive to external events.

 * @note This function is responsible for maintaining the continuous operation of the following robot's
 * control thread, allowing it to track and follow the leader or perform other relevant actions without
 * interruption.
 */
void Method::followingRobotThread(){
  while(true){
    followingRobotRun();
  }
}


/**
 * @brief Control the movement of a guider robot based on leader goals.
 *
 * This function is responsible for controlling the movement of a guider robot based on the leader's goals.
 * It allows the guider robot to navigate towards specified goals while taking into account the current
 * threading state (Threading_switch).

 * If threading is enabled (Threading_switch is true), the function performs the following steps:
 * - Pauses for 500 milliseconds to allow for synchronization.
 * - Iterates through the list of leader goals (Leader_goals) and sets the guider's goal accordingly.
 * - Calculates the trajectory to reach the goal using the 'reachGoal' function from GuiderGPS.
 * - Sends the calculated trajectory to control the guider's movement using 'Send_cmd_tb2'.

 * If threading is not enabled (Threading_switch is false), the function follows a similar process:
 * - Sets the guider's goal based on the goal at the current 'goal_index' in the list of leader goals.
 * - Calculates the trajectory for the guider to reach this goal using 'guiderReachGoal' function.
 * - Sends the trajectory to control the guider's movement using 'Send_cmd_tb2'.
 * - Checks if the guider has reached the goal using 'goal_hit', and if so, increments the goal_index.

 * @note This function is a key part of the guider robot's behavior, allowing it to follow leader goals and
 * adjust its movement based on threading settings and goal-reaching status.
 */
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

/**
 * @brief Run the control logic for the following robot.
 *
 * This function is responsible for executing the control logic for the following robot. It includes the following steps:
 * 1. Updates the robot's sensor data by calling 'Newdata' with updated image data obtained from 'Update_Robot_Image_data'.
 * 2. Determines the goal point for the following robot by calling 'findTurtlebot' on the updated sensor data.
 * 3. Sets the goal for the following robot using the determined point.
 * 4. Calculates the trajectory to reach the goal by calling 'reachGoal' on the GPS (Global Positioning System).
 * 5. Sends the calculated trajectory to control the motion of the following robot.

 * This function is essential for managing the behavior of the following robot as it tracks the goal, taking into account
 * sensor data and GPS-based navigation.

 * @note The control logic executed in this function ensures that the following robot actively pursues the goal
 * and maintains its trajectory accordingly.
 */
void Method::followingRobotRun(){

  scanData.Newdata(Update_Robot_Image_data());
  goal = scanData.findTurtlebot();
  //goal = adjustLaserData(goal, Current_Odom);

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


/**
 * @brief Adjust laser data coordinates based on robot orientation and position.
 *
 * This function adjusts the laser data coordinates to be in the robot's reference frame, considering its
 * orientation and position. It performs a coordinate transformation and applies the robot's current pose.

 * @param laser_data The original laser data in the global reference frame.
 * @param Position The current odometry position of the robot.

 * @return The adjusted laser data coordinates in the robot's reference frame.

 * The function takes the original laser data in the global reference frame and the robot's current odometry position.
 * It first extracts the robot's orientation from the odometry message and converts it to Euler angles (roll, pitch, yaw).
 * Using these angles, it performs a coordinate transformation to adjust the laser data coordinates to the robot's
 * reference frame.

 * The adjusted values are further adjusted by adding the robot's current position offset. The resulting adjusted
 * coordinates are returned.

 * @note This function is critical for aligning laser data with the robot's orientation and position, ensuring
 * accurate data interpretation for navigation and mapping.
 */
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

/**
 * @brief Read and parse goal points from a text file.
 *
 * This function reads goal points from a text file, parses the contents, and populates the 'Leader_goals' vector
 * with the extracted points. It allows you to specify the filename of the text file containing goal information.

 * @return 'true' if the goal points were successfully read and parsed, 'false' otherwise.

 * The function begins by defining the filename of the text file that contains the goal points. It attempts to open
 * the file for reading and checks for any errors during the file opening process. If the file cannot be opened, it
 * reports an error and returns 'false' as an error code.

 * If the file is successfully opened, the function reads each line from the file and parses it to extract the
 * 'x', 'y', and 'z' coordinates of a goal point. If parsing is successful, the extracted point is added to the
 * 'Leader_goals' vector. If any line cannot be parsed, an error message is displayed.

 * After reading and parsing all the goal points, the function closes the file and prints the parsed points to the console.

 * @note This function is crucial for loading and configuring goal points from a text file, enabling the robot to
 * navigate to predefined locations.
 */
bool Method::readGoal() {
    // Define the filename of the text file you want to read
    // std::string filename = "../data/Goals.TXT";


    std::string path = ros::package::getPath("turtleboi");
    path += "/data/"; //Looking at data subfolder
    std::string default_filename = path + "Goals.TXT";

    std::string filename;
    
    nh_.param<std::string>("goals", filename, default_filename);


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



//Telelop functions
/////////////////////////////////////////////////
void Method::telop(){
  
  
  cout << "You will be able to control" << endl;
  cout << "Speed via i/k." << endl;
  cout << "Steering via j/l." << endl;
  cout << "Emergency stop is o" << endl;
  cout << "Exit is ." << endl;


  double brake=0,steering=0,throttle=0;
  geometry_msgs::Twist intructions;
  
  char c;
  bool run=true;
  // Set the terminal to raw mode
  while(run) {
    system("stty raw");
    c = getchar(); 
    // terminate when "." is pressed
    system("stty cooked");
    system("clear");
    std::cout << c << " was pressed."<< std::endl;
    switch(c){
      case '.' :
          system("stty cooked");
          run=0;
          break;
      case 'i' :
          intructions.linear.x = 0;
          break;
      case 'k' :
          intructions.linear.x = 0;
          break;
      case 'o' :
          intructions.linear.x = 0;
          intructions.angular.z = 0;
          break;                
      case 'j' :
          intructions.angular.z =- 0.05;
          break;
      case 'l' :
          intructions.angular.z =+ 0.05;
          break;
      default:
          break;
    }


    if(run){
      Send_cmd_tb2(intructions);
      std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }
  }
}

void Method::telopDrive(void){
    
    while(true){    
      std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }

}


