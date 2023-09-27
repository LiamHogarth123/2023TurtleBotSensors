#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseArray.h>
#include "visualization_msgs/MarkerArray.h"
/**
 @file dataprocessing.h
 @brief This file contains the implementation of the DataProcessing class.
*/

class DataProcessing
{
    public:
    
    /**
    @brief Constructor for the DataProcessing class.
    */
    DataProcessing();

    /**
    @brief Checks if a goal point is within a cone.
    @param goals The goal point to be checked.
    @param midpoint The midpoint of the cone.
    @return Returns true if the goal is within the cone, false otherwise.
    */
    bool goalWithinCone(geometry_msgs::Point goals, geometry_msgs::Point midpoint);
    /**
    @brief Adjusts the laser data based on the car's position.
    @param laser_points The laser points to be adjusted.
    @param CarPosition The position of the car.
    @return The adjusted laser points.
    */
    bool goalWithinCone(geometry_msgs::Point goals, std::vector<geometry_msgs::Point> midpoints);

    /**
    @brief Adjusts a vector of laser data points based on the car's position.
    @param laser_points The laser points to be adjusted.
    @param CarPosition The position of the car.
    @return The adjusted laser points.
    */
    geometry_msgs::Point adjustLaserData(geometry_msgs::Point laser_points, geometry_msgs::Pose CarPosition);

    /**
    @brief Visualizes cones as markers in a marker array.
    @param cones The cones to be visualized.
    @param markerArray The marker array to add the cones to.
    @return The updated marker array.
    */
    std::vector<geometry_msgs::Point> adjustLaserData(std::vector<geometry_msgs::Point> laser_points, geometry_msgs::Pose CarPosition);

    /**
    @brief Visualizes cones as markers in a marker array (alternative version).
    @param cones The cones to be visualized.
    @param markerArray The marker array to add the cones to.
    @return The updated marker array.
    */
    visualization_msgs::MarkerArray visualiseCone(std::vector<geometry_msgs::Point> cones, visualization_msgs::MarkerArray markerArray);

    /**
    @brief Visualizes the road center as a marker in a marker array.
    @param center The center point of the road.
    @param markerArray The marker array to add the road center marker to.
    @return The updated marker array.
    */
    visualization_msgs::MarkerArray visualiseRoadCenter(geometry_msgs::Point center, visualization_msgs::MarkerArray markerArray);

    /**
    @brief Checks if a goal point is within a cone defined by multiple midpoint points.
    @param goals The goal point to check.
    @param midpoints The midpoint points defining the cone.
    @return True if the goal is within the cone, False otherwise.
    */
    visualization_msgs::MarkerArray visualisetest(std::vector<geometry_msgs::Point> cones, visualization_msgs::MarkerArray markerArray);

    int ct = 0;

};

#endif // DETECTCABINET_H


