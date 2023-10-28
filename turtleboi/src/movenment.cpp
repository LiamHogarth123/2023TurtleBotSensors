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

    // std::cout << "followerDirectDistance: " << DirectDistance << std::endl;



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
        // std::cout << "braking" << std::endl;
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

    double LINEAR_SPEED = 0.15;

    double delta_x = Goal.x - Current_Pose.pose.pose.position.x;
    double delta_y = Goal.y - Current_Pose.pose.pose.position.y;

    // std::cout << "Goalx: " << Goal.x << std::endl;
    // std::cout << "Goaly: " << Goal.y << std::endl;
    // std::cout << "x: " << Current_Pose.pose.pose.position.x << std::endl;
    // std::cout << "y: " << Current_Pose.pose.pose.position.y << std::endl;
    // std::cout << "dx: " << delta_x << std::endl;
    // std::cout << "dy: " << delta_y << std::endl;

    DirectDistance = sqrt(std::pow(delta_x,2) + std::pow(delta_y,2));
    // std::cout << "DirectDistance: " << DirectDistance << std::endl;

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
        // std::cout << "braking" << std::endl;
    }

    return Directions;
}

bool Movenment::goal_hit(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_Current_Pose){
    double delta_x = Goal.x - Current_Pose.pose.pose.position.x;
    double delta_y = Goal.y - Current_Pose.pose.pose.position.y;
    DirectDistance = sqrt(std::pow(delta_x,2) + std::pow(delta_y,2));
    if (DirectDistance <= distance_from_goal) {
        return true;
    }
    else{
        return false;
    }
} 


// Function to calculate the required angular velocity to reach the goal
double Movenment::calculateAngularVelocity() {
    double ANGULAR_SPEED = 1;
    double ANGULAR_DEAD_ZONE = 0.1; // to help prevent wobbling due to angle changes

    tf::Quaternion current_orientation;

    tf::quaternionMsgToTF(Current_Pose.pose.pose.orientation, current_orientation);

    // Normalize the quaternion
    current_orientation.normalize();

    // Calculate the heading direction vector
    tf::Vector3 heading_vector(1, 0, 0);  // Assumes the robot's heading direction is along the x-axis

    // Rotate the heading vector to the current orientation
    heading_vector = tf::quatRotate(current_orientation, heading_vector);

    // Calculate the vector to the goal
    tf::Vector3 goal_vector(Goal.x - Current_Pose.pose.pose.position.x, Goal.y - Current_Pose.pose.pose.position.y, 0);

    // Calculate the angle between the heading direction and the goal vector
    double angle = atan2(goal_vector.y(), goal_vector.x()) - atan2(heading_vector.y(), heading_vector.x());

    // Adjust the angular velocity based on the relative angle
    if (fabs(angle) < ANGULAR_DEAD_ZONE) {
        return 0.0;
    } else if (angle > 0) {
        return ANGULAR_SPEED;
    } else {
        return -ANGULAR_SPEED;
    }
    
}

void Movenment::change_stopping_distance(double value){
    distance_from_goal = value;
}