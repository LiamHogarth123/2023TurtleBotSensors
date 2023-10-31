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

/**
 * @brief Set a new movement goal and update the current pose.
 *
 * This function updates the movement goal and the current pose with the provided values.
 * It is typically used to set a new goal for a navigation system and update the robot's current pose
 * accordingly.

 * @param temp_goal The geometry_msgs::Point representing the new movement goal.
 * @param temp_Current_Pose The nav_msgs::Odometry representing the robot's current pose.

 * The function sets the new movement goal (Goal) and updates the robot's current pose (Current_Pose)
 * with the provided values.

 * @note This function is crucial for redefining the robot's navigation goal and ensuring that the
 * current pose is in sync with the robot's position and orientation.
 */
void Movenment::newGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_Current_Pose){
    Goal = temp_goal;
    Current_Pose = temp_Current_Pose;
}

/**
 * @brief Calculate the required linear and angular velocities to reach a specified goal point.
 *
 * This function performs the necessary calculations to determine the linear and angular velocities
 * required for a robot to reach a given goal point. The function takes into account the distance to
 * the goal and the angle between the robot's current orientation and the direction to the goal.

 * @return A geometry_msgs::Twist containing the calculated linear and angular velocities.

 * The function calculates the distance (DirectDistance) to the goal point, the angle (Angle) between the
 * robot's current orientation and the direction to the goal, the turning radius (radius), and the default
 * linear velocity (default_velocity). Based on these calculations, it determines the linear and angular
 * velocities required to reach the goal point.

 * Additionally, it includes conditions to handle specific situations:
 * - If the robot is close to the goal (within a specified distance), it applies braking.
 * - If the goal point is exactly at the current robot position, it applies rotation to search for the goal.
 * - If the goal is to the right or left of the robot, it adjusts the angular velocity for turning.
 * - If the goal is directly in front of the robot, turning is stopped.

 * @note This function is a critical component of robot motion planning and control, ensuring that the
 * robot moves towards its goal with the appropriate velocities and behaviors.
 */
geometry_msgs::Twist Movenment::reachGoal(){
    //do math to calculate the required linear and angular velocity to reach point
    geometry_msgs::Twist Directions; 

    DirectDistance = sqrt(std::pow(Goal.x,2) + std::pow(Goal.y,2));
    
    Angle = atan2(Goal.y,Goal.x); // check this as x and y could be flipped
    //std::cout << "Angle " << Angle << std::endl;

    // std::cout << "followerDirectDistance: " << DirectDistance << std::endl;

    theta = M_PI - (2*((M_PI/2)-Angle));

    radius = (DirectDistance/sin(theta))*sin((M_PI/2)-Angle);
    //std::cout << "radius" << std::endl;
    //std::cout << radius << std::endl;

    double default_velocity = 0.25;


    Directions.linear.x = default_velocity;

    if (DirectDistance <= 0.8) {
        Directions.linear.x = 0.1;
    } else {
        Directions.linear.x = default_velocity;
    }
    
    // For braking
    if (DirectDistance < distance_from_goal && DirectDistance > 0) {
        Directions.linear.x = 0;
        Directions.angular.z = 0;
        std::cout << "braking" << std::endl;
    }
    // for rotating the bot if it cant find turtlebot
    else if (DirectDistance == 0) {
        Directions.linear.x = 0;
        Directions.angular.z = 1.5;
        //std::cout << "here1" << std::endl;
    }
    // changing the angular velocity to turn right towards the guider
    else if (Goal.x > 0) {
        Directions.angular.z = default_velocity/radius;
        //std::cout << "here2" << std::endl;
    }
    // changing the angular velocity to turn left towards the guider
    else if (Goal.x < 0) {
        Directions.angular.z = - default_velocity/radius;
        //std::cout << "here3" << std::endl;
    }
    
    // if guider is directly in front of the follower (Goal.x == 0) turning is stopped
    else {
        Directions.angular.z = 0;
        //std::cout << "here4" << std::endl;
    }
    
    return Directions;
    
}

/**
 * @brief Calculate the linear and angular velocities for a guider robot to reach its goal.
 *
 * This function calculates the linear and angular velocities required for a guider robot to reach
 * a specified goal point based on the current robot pose and the goal coordinates.

 * @return A geometry_msgs::Twist message containing the calculated linear and angular velocities.

 * The function starts by defining a constant LINEAR_SPEED for the linear velocity. It then calculates
 * the difference in x (delta_x) and y (delta_y) coordinates between the current robot pose and the goal.
 * Using these differences, it computes the direct distance (DirectDistance) to the goal point.

 * The function also calls the 'calculateAngularVelocity' function to determine the appropriate angular
 * velocity for the guider robot to align with the goal direction.

 * Based on the calculated angular velocity and the predefined linear speed, it creates a geometry_msgs::Twist
 * message (Directions) to control the robot's motion.

 * Additionally, it checks the distance to the goal, and if the guider robot is within a specified range of
 * the goal (distance_from_goal), it applies braking by setting linear and angular velocities to zero.

 * @note This function is crucial for controlling the guider robot's movement as it navigates towards the goal
 * while maintaining the correct orientation.
 */
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

/**
 * @brief Check if a goal has been reached based on current robot pose.
 *
 * This function determines whether a specified goal has been reached by comparing the distance between
 * the goal point and the current robot pose with a predefined threshold (distance_from_goal).

 * @param temp_goal The geometry_msgs::Point representing the goal to be checked.
 * @param temp_Current_Pose The nav_msgs::Odometry containing the current robot pose.

 * @return True if the goal has been reached, false otherwise.

 * The function calculates the difference in x (delta_x) and y (delta_y) coordinates between the goal and
 * the current robot pose. It then computes the direct distance (DirectDistance) to the goal point using
 * these differences.

 * If the DirectDistance is less than or equal to the predefined threshold (distance_from_goal), the function
 * returns true, indicating that the goal has been reached. Otherwise, it returns false.

 * @note This function is useful for checking whether the robot has reached its intended goal, allowing
 * for decision-making in navigation and control systems.
 */
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


/**
 * @brief Calculate the angular velocity required to align with the goal direction.
 *
 * This function calculates the angular velocity required to align the robot's orientation with the
 * direction of the goal point. It is essential for controlling the robot's rotation to face the goal.

 * @return The calculated angular velocity (positive for clockwise, negative for counterclockwise).

 * The function starts by defining constants for ANGULAR_SPEED (maximum angular velocity) and ANGULAR_DEAD_ZONE
 * (a threshold to prevent wobbling due to small angle changes).

 * It extracts the current orientation of the robot from the provided nav_msgs::Odometry, normalizes the quaternion,
 * and calculates the heading direction vector (assumes the robot's heading is along the x-axis).

 * The function then calculates the vector from the robot's current position to the goal and determines the angle
 * between the heading direction and the goal vector.

 * Based on the calculated angle, the function adjusts the angular velocity:
 * - If the angle is within the ANGULAR_DEAD_ZONE, it returns a zero angular velocity to prevent excessive rotation.
 * - If the angle is positive (indicating a clockwise rotation is needed), it returns a positive ANGULAR_SPEED.
 * - If the angle is negative (indicating a counterclockwise rotation is needed), it returns a negative ANGULAR_SPEED.

 * @note This function plays a crucial role in achieving the correct orientation to face the goal during navigation
 * and motion control.
 */
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

/**
 * @brief Change the stopping distance for reaching a goal.
 *
 * This function allows you to change the stopping distance, which is the threshold distance used
 * to determine when a goal has been reached. By modifying this value, you can adjust the proximity
 * at which the robot considers a goal as reached.

 * @param value The new stopping distance value to be set.

 * The function takes a new stopping distance value as an input parameter and assigns it to the
 * internal 'distance_from_goal' variable. This updated value will be used to check if a goal has
 * been reached when using the 'goal_hit' function.

 * @note Changing the stopping distance can impact the behavior of goal-reaching operations and
 * allows you to customize when the robot considers a goal as successfully reached.
 */
void Movenment::change_stopping_distance(double value){
    distance_from_goal = value;
}