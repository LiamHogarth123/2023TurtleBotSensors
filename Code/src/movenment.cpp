#include "dataprocessing.h"
#include <algorithm>
#include <numeric>

#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>
/**
 @file dataprocessing.h
 @brief This file contains the implementation of the DataProcessing class.
*/


DataProcessing::DataProcessing(){

}

bool DataProcessing::goalWithinCone(geometry_msgs::Point goals, geometry_msgs::Point midpoint){
    if (pow(pow(goals.x-midpoint.x,2)+ pow(goals.y-midpoint.y,2),0.5) < 6){
        return true;
    }
    else {
        return false;
    }
}


geometry_msgs::Point DataProcessing::adjustLaserData(geometry_msgs::Point laser_points, geometry_msgs::Pose CarPosition){
    geometry_msgs::Point adjustedValues;
    geometry_msgs::Point offset;

    auto yaw = tf::getYaw(CarPosition.orientation);

    offset.y = 3.5 * sin(yaw);
    offset.x = 3.5 * cos(yaw);
    
    adjustedValues.x =  laser_points.x* cos(yaw) - laser_points.y * sin(yaw)+ CarPosition.position.x + offset.x;
    adjustedValues.y = laser_points.x * sin(yaw) + laser_points.y * cos(yaw)+ CarPosition.position.y + offset.y;

    return adjustedValues;
}


std::vector<geometry_msgs::Point> DataProcessing::adjustLaserData(std::vector<geometry_msgs::Point> laser_points, geometry_msgs::Pose CarPosition){
    std::vector<geometry_msgs::Point> adjustLaserData;

    // The transform converts laser points from local to global coordinates. Extract quaternion, yaw, and translation.
    auto quat = CarPosition.orientation;
    auto yaw = tf::getYaw(quat);
    auto origin = CarPosition.position;
    geometry_msgs::Point offset;
    offset.y = 3.60 * sin(yaw);
    offset.x = 3.60 * cos(yaw);


    for (const auto& point : laser_points) {
        geometry_msgs::Point transformedPoint;
        transformedPoint.x = point.x * cos(yaw) - point.y * sin(yaw) + origin.x + offset.x;
        transformedPoint.y = point.x * sin(yaw) + point.y * cos(yaw) + origin.y + offset.y;
        transformedPoint.z = point.z;
        adjustLaserData.push_back(transformedPoint);
    }

    return adjustLaserData;
}


visualization_msgs::MarkerArray DataProcessing::visualiseCone(std::vector<geometry_msgs::Point> cones, visualization_msgs::MarkerArray markerArray){
    for (auto pt:cones){
        visualization_msgs::Marker marker;

        //We need to set the frame
        // Set the frame ID and time stamp.
        marker.header.frame_id = "world";
        //single_marker_person.header.stamp = ros::Time();
        marker.header.stamp = ros::Time::now();

        //We set lifetime (it will dissapear in this many seconds)
        marker.lifetime = ros::Duration(1000.0); //zero is forever

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "goals"; //This is namespace, markers can be in diofferent namespace
        marker.id = ct++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

        // The marker type
        marker.type = visualization_msgs::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pt.x;
        marker.pose.position.y = pt.y;
        marker.pose.position.z = pt.z;


        //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;


        // Set the scale of the marker -- 1m side
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;

        //Let's send a marker with color (green for reachable, red for now)
        std_msgs::ColorRGBA color;
        color.a=0.5;//a is alpha - transparency 0.5 is 50%;
        color.r=0;
        color.g=1.0;
        color.b=0;

        marker.color = color;

        markerArray.markers.push_back(marker);
    }

    // pub_.publish(markerArray);

    return markerArray;
}



visualization_msgs::MarkerArray DataProcessing::visualisetest(std::vector<geometry_msgs::Point> cones, visualization_msgs::MarkerArray markerArray){
for (auto pt:cones){
        visualization_msgs::Marker marker;

        //We need to set the frame
        // Set the frame ID and time stamp.
        marker.header.frame_id = "world";
        //single_marker_person.header.stamp = ros::Time();
        marker.header.stamp = ros::Time::now();

        //We set lifetime (it will dissapear in this many seconds)
        marker.lifetime = ros::Duration(1000.0); //zero is forever

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "goals"; //This is namespace, markers can be in diofferent namespace
        marker.id = ct++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

        // The marker type
        marker.type = visualization_msgs::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pt.x;
        marker.pose.position.y = pt.y;
        marker.pose.position.z = pt.z;


        //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;


        // Set the scale of the marker -- 1m side
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;

        //Let's send a marker with color (green for reachable, red for now)
        std_msgs::ColorRGBA color;
        color.a=0.5;//a is alpha - transparency 0.5 is 50%;
        color.r=1.0;
        color.g=0.0;
        color.b=0;

        marker.color = color;

        markerArray.markers.push_back(marker);
    }

    // pub_.publish(markerArray);

    return markerArray;
}


visualization_msgs::MarkerArray DataProcessing::visualiseRoadCenter(geometry_msgs::Point center, visualization_msgs::MarkerArray markerArray){
    visualization_msgs::Marker marker;

        //We need to set the frame
        // Set the frame ID and time stamp.
        marker.header.frame_id = "world";
        //single_marker_person.header.stamp = ros::Time();
        marker.header.stamp = ros::Time::now();

        //We set lifetime (it will dissapear in this many seconds)
        marker.lifetime = ros::Duration(1000.0); //zero is forever

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "road"; //This is namespace, markers can be in diofferent namespace
        ct ++;
        marker.id = ct; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

        // The marker type
        marker.type = visualization_msgs::Marker::CUBE;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position = center;
        

        //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;


        // Set the scale of the marker -- 1m side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        //Let's send a marker with color (green for reachable, red for now)
        std_msgs::ColorRGBA color;
        color.a=0.5;//a is alpha - transparency 0.5 is 50%;
        color.r=1.0;
        color.g=0.0;
        color.b=0;

        marker.color = color;

        markerArray.markers.push_back(marker);
    
   

    // pub_.publish(markerArray);

    return markerArray;
}



bool DataProcessing::goalWithinCone(geometry_msgs::Point goals, std::vector<geometry_msgs::Point> midpoints){
    for (int i =0; i < midpoints.size(); i++){
        if (pow(pow(goals.x-midpoints.at(i).x,2)+ pow(goals.y-midpoints.at(i).y,2),0.5) < 15){
            return true;
        }
        else {  
            
        }
    }
    return false;
}

