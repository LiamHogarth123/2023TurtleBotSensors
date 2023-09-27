#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseArray.h>

//#include "laserprocessing.h"
class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::LaserScan laserScan);

   /*! 
   * @brief This function find the midPoints between the cones discovered in the scan data
   *
   * @return This function returns the location of the midpoint as a pose
   */
  geometry_msgs::Point detectMidPointBasic();

  /*! 
   * @brief This function find the midPoints between the cones discovered in the scan data
   *
   * @return This function returns the location of the midpoint as a pose
   */
  geometry_msgs::Point detectMidPointAdvance();


   /*! 
   * @brief This function find the midPoints between the cones discovered in the scan data
   *
   * @return This function returns the location of the midpoint as a pose
   */
  std::vector< geometry_msgs::Point> detectMidPointAdvancetest();

  


    /*! 
   * @brief this function looks at the laser can data within the close range of the car. It check whether there is a truck in front of the car within a range which the car would collide
   * @return This function returns a boolean value if the truck is on a collision course
   */  
  bool isTruck(double track, double wwheel_base);


  /// @brief This function finds all of the cones on the track and checks if it is a cone or truck.
  /// @return This function returns a vector pair of each cone location and index within the laserRange vector.
  std::vector<std::pair<geometry_msgs::Point, int>> findAllCones();
  


  /// @brief This function intention is to take the data from findAllCones() and return the cone positions
  /// @return This function returns a vector of all the cone positions as points.
  std::vector<geometry_msgs::Point> GetCones();


  
  
  
  
  /**

  @brief Converts polar coordinates to Cartesian coordinates.
  @param[in] index - index value.
  @return The Cartesian coordinates as a point.
  */
  geometry_msgs::Point polarToCart(unsigned int index);
  
  /**
  @brief Calculates the angle between two points.
  @param[in] p1 - first point.
  @param[in] p2 - second point.
  @return The angle between the two points.
  */
  double angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2);
  
  /**
  @brief Updates the laser scan data.
  @param[in] laserScan - new laser scan data.
  */
  void newScan(sensor_msgs::LaserScan laserScan);


private:

private:
    sensor_msgs::LaserScan laserScan_;
    geometry_msgs::Pose SecondMidPoint;
    std::vector <geometry_msgs::Point> lastMidPoints;
    std::vector <geometry_msgs::Point> thirdlastMidPoints;
};

#endif // DETECTCABINET_H
