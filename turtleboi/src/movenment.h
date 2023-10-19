#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H


#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
/**
 @file dataprocessing.h
 @brief This file contains the implementation of the DataProcessing class.
*/

class Movenment
{
    public:
    
    /**
    @brief Constructor for the DataProcessing class.
    */
    Movenment();

    void newGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_Current_Pose );

    geometry_msgs::Twist reachGoal();

    bool goal_hit(nav_msgs::Odometry temp_Current_Pose);





    //parameters
    private:
    geometry_msgs::Point Goal;
    nav_msgs::Odometry Current_Pose;

    //Calculation variable declaration
    double Deta_x;
    double Deta_y;
    double DirectDistance;
    double Angle;
    double theta;
    double radius;
    double default_velocity;


};

#endif // DETECTCABINET_H


