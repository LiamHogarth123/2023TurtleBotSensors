#include "sensorprocessing.h"


using namespace std;

Sensorprocessing::Sensorprocessing(){   
    Turtlebot_width = 0.2;
}

/**
 * @brief Update the sensor data with new RobotData.
 *
 * This function updates the internal sensor data (Image_data) with the provided RobotData.
 * It is used to refresh the sensor data with new information, typically received from a robot or
 * sensor source.

 * @param temp_data The RobotData containing the new sensor information to be used for processing.

 * The function assigns the contents of temp_data to the internal Image_data variable, effectively
 * updating the sensor data with the latest information.

 * @note This function is used to keep the sensor data up to date and is typically called when new
 * data is received from the robot or sensor source.
 */
void Sensorprocessing::Newdata(RobotData temp_data){
    Image_data = temp_data;
}


/**
 * @brief Find the position of the Turtlebot in the laser scan data.
 *
 * This function analyzes laser scan data to locate the position of the Turtlebot by identifying
 * segments in the data that match certain criteria. The function returns the estimated position
 * of the Turtlebot in Cartesian coordinates.
 *
 * @return A geometry_msgs::Point representing the estimated position of the Turtlebot.
 *
 * The function iterates through the laser scan data to identify segments where the range values
 * meet specific conditions. A segment is a continuous set of laser scan points with small variations
 * in range values.
 *
 * The algorithm works as follows:
 * - For each laser scan reading, it calculates the difference in range values between the current
 *   and the previous reading to identify segments.
 * - If a non-infinite range value is found and the distance between scan points is less than or equal
 *   to 0.3, the function starts recording a segment.
 * - The function then calculates the center point of the segment.
 * - It checks if the segment's width is within the range of (Turtlebot_width - 0.18) to (Turtlebot_width + 0.1).
 * - If the segment width meets the criteria, the center point is considered as the estimated position
 *   of the Turtlebot.
 *
 * @note The function relies on the polarToCart() function to convert polar coordinates to Cartesian
 *       coordinates. It assumes that Image_data contains valid laser scan data.
 */
geometry_msgs::Point Sensorprocessing::findTurtlebot(){
    geometry_msgs::Point turtlebot_position; //float for the average range of that segments and int is the center point of said segments


    for (int i = 1; i < Image_data.laserScan.ranges.size(); i++) { // for all readings
        float currentRange = Image_data.laserScan.ranges.at(i);
        float prevRange = Image_data.laserScan.ranges.at(i - 1);
        float Distance_Between_scanpoint = fabs(currentRange - prevRange);

        if(!std::isinf(Image_data.laserScan.ranges.at(i))){ // if the number isn't infinity
            int i_start = i;
          
            while(!std::isinf(Image_data.laserScan.ranges.at(i))&& Distance_Between_scanpoint <= 0.3){
            
                if (i == Image_data.laserScan.ranges.size()-1){
                    break;
                }
                i++;
                currentRange = Image_data.laserScan.ranges.at(i);
                prevRange = Image_data.laserScan.ranges.at(i - 1);
                Distance_Between_scanpoint = fabs(currentRange - prevRange);
            } 
            
            int i_end = i-1;
            int i_center = i_start+(i_end-i_start)/2;

            geometry_msgs::Point pt1 =polarToCart(i_start);
            geometry_msgs::Point pt2 =polarToCart(i_end);

            double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
            
            if (distance < (Turtlebot_width+0.1) && distance > (Turtlebot_width - 0.18)){
                

                geometry_msgs::Point segment_center;

                segment_center.x = ((pt1.x + pt2.x) / 2);
                segment_center.y = ((pt1.y + pt2.y) / 2);
                
                turtlebot_position = segment_center;
            
            
            }
        }
    }

return turtlebot_position;

}

/**
 * @brief Convert a polar coordinate to a Cartesian point.
 *
 * This function takes an index as input and calculates the Cartesian coordinates
 * (x, y) from the polar coordinates (range, angle) at the specified index.
 *
 * @param index The index of the laser scan data point to be converted.
 * @return A geometry_msgs::Point representing the Cartesian coordinates.
 *
 * @note The function assumes that Image_data contains laser scan data with valid
 *       angle_min, angle_increment, and ranges values.
 */
geometry_msgs::Point Sensorprocessing::polarToCart(unsigned int index)
{
    
    float angle = Image_data.laserScan.angle_min + Image_data.laserScan.angle_increment*index;// + angle_range/2;
    float range = Image_data.laserScan.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}


