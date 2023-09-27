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
  Sensorprocessing(RobotData New_data);


  RobotData Image_data;

};

#endif // DETECTCABINET_H
