#include "movenment.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
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

    DirectDistance = sqrt(std::pow(Goal.x,2) + std::pow(Goal.y,2));
    
    Angle = atan2(Goal.y,Goal.x); // check this as x and y could be flipped
    //std::cout << "Angle " << Angle << std::endl;

    //std::cout << "DirectDistance: " << DirectDistance << std::endl;



    theta = M_PI - (2*((M_PI/2)-Angle));

    radius = (DirectDistance/sin(theta))*sin((M_PI/2)-Angle);
    //std::cout << "radius" << std::endl;
    //std::cout << radius << std::endl;

    double default_velocity = 0.2;


    Directions.linear.x = default_velocity;
    
    // For braking
    if (DirectDistance < distance_from_goal && DirectDistance > 0) {
        Directions.linear.x = 0;
        Directions.angular.z = 0;
        std::cout << "braking" << std::endl;
    }
    // for rotating the bot if it cant find turtlebot
    else if (DirectDistance == 0) {
        Directions.linear.x = 0;
        Directions.angular.z = 1;
        //std::cout << "here1" << std::endl;
    }
    // changing the angular velocity to turn right towards the guider
    else if (Goal.x > 0) {
        Directions.angular.z = default_velocity/radius;
        //std::cout << "here2" << std::endl;
    }
    // changing the angular velocity to turn left towards the guider
    else if (Goal.x < 0) {
        Directions.angular.z = -default_velocity/radius;
        //std::cout << "here3" << std::endl;
    }
    // if guider is directly in front of the follower (Goal.x == 0) turning is stopped
    else {
        Directions.angular.z = 0;
        //std::cout << "here4" << std::endl;
    }


   
    
    return Directions;
    
}

geometry_msgs::Twist Movenment::guiderReachGoal() {
    
    //geometry_msgs::Twist Directions; 

    double LINEAR_SPEED = 0.2;

    double delta_x = Goal.x - Current_Pose.pose.pose.position.x;
    double delta_y = Goal.y - Current_Pose.pose.pose.position.y;

    std::cout << "Goalx: " << Goal.x << std::endl;
    std::cout << "Goaly: " << Goal.y << std::endl;
    std::cout << "x: " << Current_Pose.pose.pose.position.x << std::endl;
    std::cout << "y: " << Current_Pose.pose.pose.position.y << std::endl;
    std::cout << "dx: " << delta_x << std::endl;
    std::cout << "dy: " << delta_y << std::endl;

    DirectDistance = sqrt(std::pow(delta_x,2) + std::pow(delta_y,2));
    std::cout << "DirectDistance: " << DirectDistance << std::endl;

    // Calculate the angular velocity
    double angular_velocity = calculateAngularVelocity();

    // Create a Twist message to control the robot
    geometry_msgs::Twist Directions;
    Directions.angular.z = angular_velocity;

    // Adjust linear velocity based on the distance to the goal
    Directions.linear.x = LINEAR_SPEED;   
    if (DirectDistance <= distance_from_goal) {
        Directions.linear.x = 0;
        Directions.angular.z = 0;
        std::cout << "braking" << std::endl;
    }
    // else if (DirectDistance > LINEAR_SPEED) {
    //     Directions.linear.x = LINEAR_SPEED;
    // } else {
    //     Directions.linear.x = DirectDistance;
    // }


    
    return Directions;
}

void Movenment::change_stopping_distance(double value){
    distance_from_goal = value;
}

// Function to calculate the required angular velocity to reach the goal
double Movenment::calculateAngularVelocity() {
    double ANGULAR_SPEED = 1;
    
    geometry_msgs::Quaternion current_orientation = Current_Pose.pose.pose.orientation;

    tf::Quaternion current_quaternion;
        // Normalize the quaternion
    current_quaternion.normalize();
    tf::quaternionMsgToTF(current_orientation, current_quaternion);



    tf::Vector3 goal_vector(Goal.x - Current_Pose.pose.pose.position.x, Goal.y - Current_Pose.pose.pose.position.y, 0);

    // Calculate the relative yaw angle between the current orientation and the goal vector
    double yaw_angle = tf::getYaw(current_quaternion.inverse() * goal_vector);

    // Adjust the angular velocity based on the relative angle
    if (yaw_angle > 0) {
        return ANGULAR_SPEED;
    } else {
        return -ANGULAR_SPEED;
    }
}
