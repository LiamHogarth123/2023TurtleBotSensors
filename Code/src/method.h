/**
 @file sample.h
 @brief Header file for the Sample class.
*/

#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

//Keep only the headers needed
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"

//We include header of anotehr class we are developing
#include "laserprocessing.h"
#include "dataprocessing.h"


// @brief Platform status enumeration.
namespace Ackerman{
    typedef enum {
      IDLE, /*!< Stationary with no goal */
      RUNNING, /*!< Executing a motion */
    } PlatformStatus; /*!< Platform Status */
}

/**
 @class Sample
 @brief A class representing a sample.
*/
class Sample
{
public:
  /*
  @brief Default constructor.
  @param nh The ROS node handle.
  */
  Sample(ros::NodeHandle nh);
 
 /**
 @brief Destructor.
 */
  ~Sample();



   /*! @brief seperate thread.
  *
  *  The main processing thread that will run continously and utilise the data
  *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
  */
  void seperateThread();

  

//Ros Functions////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  /**
 * @brief Laser callback.
 * @param msg The laser scan message.
 */
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

/**
 * @brief Goals callback.
 * @param msg The pose array message.
 */
void goalsCallback(const geometry_msgs::PoseArray::ConstPtr& msg);


/**
 * @brief Request service callback.
 * @param req The request.
 * @param res The response.
 * @return bool - Will return true to indicate the request succeeded.
 */
bool request(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

/**
 * @brief Request advance service callback.
 * @param req The request.
 * @param res The response.
 * @return bool - Will return true to indicate the request succeeded.
 */
bool requestadvance(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);



  


//Ros Variables //////////////////////////////////////////////////////////////////////////////////////////
private:


  ros::NodeHandle nh_;//Node handle for communication

  ros::Publisher pub_;//! Visualisation Marker publisher
  ros::Publisher pubBrakeCmd_;
  ros::Publisher pubSteering;
  ros::Publisher pubThrottle;
  ros::Publisher Cone_pub;

  std::mutex odoMtx_;
  std::mutex laserDataMtx_;
  std::mutex goalMtx_;



  ros::Subscriber sub1_;  // Few subscribers
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;

  ros::ServiceServer service_; // Incoming service

  LaserProcessing* laserProcessingPtr_; //! Pointer to Laser Object

  //Call back variable storage
  geometry_msgs::Pose CarPosition;

  double speed;
  bool run;

  sensor_msgs::LaserScan laserData_;

 

  geometry_msgs::PoseArray goals; 
  bool goals_set;

  
  


  //Ackerman Functions////////////////////////////////////////////////////////////////////////////////////////////////////
  public:
  /**
  @brief Check the origin to the destination.
  @param origin The origin pose.
  @param goal The goal point.
  @param distance The calculated distance.
  @param steering_angle_ The calculated steering angle.
  @param estimatedGoalPose The estimated goal pose.
  @return bool Returns true if the check is successful.
  */
  bool checkOriginToDestination(geometry_msgs::Pose origin,geometry_msgs::Point goal, double &distance, double &steering_anlge_, geometry_msgs::Pose estimatedGoalPose);

  /**
  @brief Send command signals.
  @param brake The brake value.
  @param steering_angle_ The steering angle value.
  @param throttle The throttle value.
  */
  void sendCmd(double brake, double steering_angle_, double throttle);


  /**
  @brief Check if all goals are reached.
  @return bool Returns true if all goals are reached.
  */
  bool reachGoals();

  /**
  @brief Control the platform.
  */
  void control();

  /**
  @brief Get the platform status. intended to be used for testing purposes.
  @return Ackerman::PlatformStatus The platform status.
  */
  Ackerman::PlatformStatus getStatus();


  //Ackerman Vairable//////////////////////////////////////////
  private:
    geometry_msgs::Pose goal_;
    unsigned int ct = 0;
    double tolerance = 0.7;
    geometry_msgs::Pose estimatedGoalPose;
    double steering_ratio;
    double lock_to_lock_Revs;
    double max_steering_Angle;
    double track;
    double wheel_radius;
    double wheel_base;
    geometry_msgs::Point Current_Position;
    double max_brake_torque;
    double default_throttle; //(top speed 2.91)
    double average_velocity;
    
    double Caroffset;
    
    double time;
    double distance; 
    double steering_angle;
    double Heading_diff;
    geometry_msgs::Point truck_start; 
    geometry_msgs::Point truck_finish;
    Ackerman::PlatformStatus status_;
 

  //flags
  private:
    bool goalSet_ = true;
    bool executeGoals_ = true;
    bool all_goals_reached = false;
    bool AllGoalReached = false;
    bool basic = true; 

    std::vector<geometry_msgs::Point> FoundCones;

};

#endif // SAMPLE_H