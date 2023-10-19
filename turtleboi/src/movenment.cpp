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

Movenment::Movenment(){
    default_velocity = 0.1;
}

void Movenment::newGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_Current_Pose){
    Goal = temp_goal;
    Current_Pose = temp_Current_Pose;
}

geometry_msgs::Twist Movenment::reachGoal(){
    //do math to caclule the required linear and angular velocity to reach point
    geometry_msgs::Twist Directions;

    Deta_x = Goal.x - Current_Pose.pose.pose.position.x;
    Deta_y = Goal.y - Current_Pose.pose.pose.position.y;

    

    DirectDistance = sqrt(std::pow(Deta_x,2) + std::pow(Deta_y,2));
    
    Angle = atan2(Deta_y,Deta_x); // check this as x and y could be flipped



    theta = M_PI - (2*((M_PI/2)-Angle));

    radius = (DirectDistance/sin(theta))*sin((M_PI/2)-Angle);
  

    double default_velocity = 0.1;


    Directions.linear.x = default_velocity;
    
    if (DirectDistance < 1) {
        Directions.linear.x = 0;
        Directions.angular.z = 0;
        std::cout << "braking" << std::endl;
    }
    else {
        if (Deta_x < 0){
            Directions.angular.z = -default_velocity/radius;  
        }
        else {
            Directions.angular.z = default_velocity/radius;
        }
    }

   
    
    return Directions;
    
}

// bool Movenment::goal_hit(nav_msgs::Odometry temp_Current_Pose){
//     double Deta_x = Goal.x-Current_Pose.pose.pose.position.x;
//     double Deta_y = Goal.y - Current_Pose.pose.pose.position.y;

//     double DirectDistance = sqrt(std::pow(Deta_x,2) + std::pow(Deta_y,2));
//     std::cout << DirectDistance << std::endl;
//     if (DirectDistance < 1){
//         return false;
//     }
//     else {
//         return true;
//     }
// }
