#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

// Define a structure that includes camera data and laser scan data
struct RobotData {
    sensor_msgs::Image rgbImage;    // Camera RGB image data
    sensor_msgs::Image depthImage;  // Camera depth image data
    sensor_msgs::LaserScan laserScan;  // Laser scan data

    // Constructor to initialize the structure
    RobotData() {
        // Initialize the camera data fields
        // Assuming that rgbImage and depthImage are of the appropriate dimensions and settings
        // Initialize the laser scan data fields
        // Assuming that laserScan is appropriately configured
    }
};

#endif // ROBOT_DATA_H
