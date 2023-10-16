#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>

#include <math.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseArray.h>
#include <image_data_struct.h>


//#include "laserprocessing.h"
class Sensorprocessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  Sensorprocessing();

  geometry_msgs::Point CalculateMidPoint();

  void Newdata(RobotData temp_data);

  geometry_msgs::Point polarToCart(unsigned int index);

  geometry_msgs::Point findTurtlebot();



  private:
  RobotData Image_data;
  double Turtlebot_width;

};



#endif // DETECTCABINET_H
