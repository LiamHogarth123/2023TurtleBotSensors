#include "movenment.h"
#include <algorithm>
#include <numeric>

#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>
/**
 @file dataprocessing.h
 @brief This file contains the implementation of the DataProcessing class.
*/


Movenment::Movenment(geometry_msgs::Point temp_goal){
    Goal = temp_goal;
}

void Movenment::newGoal(geometry_msgs::Point temp_goal){
    Goal = temp_goal;
}

geometry_msgs::Twist Movenment::Cacluation(){
    //do math to caclule the required linear and angular velocity to reach point
    

}