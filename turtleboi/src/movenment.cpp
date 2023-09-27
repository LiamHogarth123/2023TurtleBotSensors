#include "movenment.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>
/**
 @file dataprocessing.h
 @brief This file contains the implementation of the DataProcessing class.
*/


Movenment::Movenment(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_Current_Pose){
    Goal = temp_goal;
    Current_Pose = temp_Current_Pose;
}

void Movenment::newGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_Current_Pose){
    Goal = temp_goal;
    Current_Pose = temp_Current_Pose;
}

geometry_msgs::Twist Movenment::Cacluation(){
    //do math to caclule the required linear and angular velocity to reach point
    geometry_msgs::Twist Directions;

    double Deta_x = Goal.x-Current_Pose.pose.pose.position.x;
    double Deta_y = Goal.y - Current_Pose.pose.pose.position.y;
    double DirectDistance = sqrt(std::pow(Deta_x,2) + std::pow(Deta_y,2));
    double Angle = atan2(Deta_x,Deta_y); // check this as x and y could be flipped

    return Directions;
    
}