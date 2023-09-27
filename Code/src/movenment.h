#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H


#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
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
    Movenment(geometry_msgs::Point temp_goal);

    void newGoal(geometry_msgs::Point temp_goal);

    geometry_msgs::Twist Cacluation();

    //parameters
    private:
    geometry_msgs::Point Goal;


};

#endif // DETECTCABINET_H


