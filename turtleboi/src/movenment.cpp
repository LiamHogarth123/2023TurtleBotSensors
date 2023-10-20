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
    distance_from_goal =0.5;

}

void Movenment::newGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_Current_Pose){
    Goal = temp_goal;
    Current_Pose = temp_Current_Pose;
}

geometry_msgs::Twist Movenment::reachGoal(){
    //do math to caclule the required linear and angular velocity to reach point
    geometry_msgs::Twist Directions;

    Deta_x = Goal.x;// - Current_Pose.pose.pose.position.x;
    Deta_y = Goal.y;// - Current_Pose.pose.pose.position.y;

    

    DirectDistance = sqrt(std::pow(Deta_x,2) + std::pow(Deta_y,2));
    
    Angle = atan2(Deta_y,Deta_x); // check this as x and y could be flipped
    //std::cout << "Angle" << std::endl;
    //std::cout << Angle << std::endl;

    //std::cout << "DirectDistance" << std::endl;
    //std::cout << DirectDistance << std::endl;



    theta = M_PI - (2*((M_PI/2)-Angle));

    radius = (DirectDistance/sin(theta))*sin((M_PI/2)-Angle);
    //std::cout << "radius" << std::endl;
    //std::cout << radius << std::endl;

    double default_velocity = 0.2;


    Directions.linear.x = default_velocity;
    
    if (DirectDistance < distance_from_goal && DirectDistance > 0) {
        Directions.linear.x = 0;
        Directions.angular.z = 0;
        std::cout << "braking" << std::endl;
    }
    else if (DirectDistance == 0) {
        Directions.linear.x = 0;
        Directions.angular.z = 1;
        //std::cout << "here1" << std::endl;
    }
    else if (Deta_x > 0) {
        Directions.angular.z = default_velocity/radius;
        //std::cout << "here2" << std::endl;
    }
    else if (Deta_x < 0) {
        Directions.angular.z = -default_velocity/radius;
        //std::cout << "here3" << std::endl;
    }
    else {
        Directions.angular.z = 0;
        //std::cout << "here4" << std::endl;
    }


   
    
    return Directions;
    
}

void Movenment::change_stopping_distance(double value){
    distance_from_goal = value;
}

