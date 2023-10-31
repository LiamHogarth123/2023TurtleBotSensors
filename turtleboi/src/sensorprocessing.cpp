#include "sensorprocessing.h"


using namespace std;

Sensorprocessing::Sensorprocessing(){   
    
    Turtlebot_min = 0.02;
    Turtlebot_max = 0.08;
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
 * @brief Print laser scan specifications.
 *
 * This function prints the specifications of a laser scan, including the minimum angle,
 * maximum angle, and angle increment. It is used to display important parameters of
 * the laser scan for debugging or informational purposes.
 */
void Sensorprocessing::PrintLaserSpec(){
    
    std::cout << "min" << std::endl;
    std::cout << Image_data.laserScan.angle_min << std::endl;
    std::cout << "max" << std::endl;
    std::cout << Image_data.laserScan.angle_max << std::endl;
    std::cout << "increment" << std::endl;
    std::cout << Image_data.laserScan.angle_increment << std::endl;
}


/**
 * @brief This the debugging expirenmentation model of the find turtlebot function seen below
 * @return A geometry_msgs::Point representing the estimated position of the Turtlebot.
 */
geometry_msgs::Point Sensorprocessing::findTurtlebottesting(){
    geometry_msgs::Point turtlebot_position; //float for the average range of that segments and int is the center point of said segments
    std::vector<geometry_msgs::Point> pointVector;

    for (int i = 1; i < Image_data.laserScan.ranges.size(); i++) { // for all readings
        float currentRange = Image_data.laserScan.ranges.at(i);
        float prevRange = Image_data.laserScan.ranges.at(i - 1);
        float Distance_Between_scanpoint = fabs(currentRange - prevRange);

        if(!std::isinf(Image_data.laserScan.ranges.at(i))){ // if the number isn't infinity
            int i_start = i;
          
            while(!std::isinf(Image_data.laserScan.ranges.at(i))&& Distance_Between_scanpoint <= 0.3){
                
                // std::cout << "laserscan range" << std::endl;
                // std::cout << Image_data.laserScan.ranges.at(i) << std::endl;   

                if (i == Image_data.laserScan.ranges.size()-1){
                    break;
                }
                
                currentRange = Image_data.laserScan.ranges.at(i);
                prevRange = Image_data.laserScan.ranges.at(i - 1);
                Distance_Between_scanpoint = fabs(currentRange - prevRange);
                i++;
            } 
            
            int i_end = i-1;
            int i_center = i_start+(i_end-i_start)/2;

            geometry_msgs::Point pt1 =polarToCart(i_start);
            geometry_msgs::Point pt2 =polarToCart(i_end);


            std::cout << "I start" << std::endl;
            std::cout << i_start << std::endl;
            std::cout << i_end << std::endl;
            
            std::cout << "I end" << std::endl;
        
            std::cout << "laserscan points" << std::endl;
            std::cout << pt1.x << std::endl;
            std::cout << pt1.y << std::endl;
            std::cout << pt2.x << std::endl;
            std::cout << pt2.y << std::endl;
            std::cout << "laserscan points end" << std::endl;

            double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);

                std::cout << "distance between scan" << std::endl;
                std::cout << distance << std::endl;
            
            if (distance >= Turtlebot_min && distance < Turtlebot_max){
                std::cout << "TURTLEBOT FOUND METHOD 1 ---" << std::endl;
                

                geometry_msgs::Point segment_center;

                segment_center.x = ((pt1.x + pt2.x) / 2);
                segment_center.y = ((pt1.y + pt2.y) / 2);
                
                turtlebot_position = segment_center;
                pointVector.insert(pointVector.begin(), segment_center);

                double DirectDistance2 = sqrt(std::pow(segment_center.x,2) + std::pow(segment_center.y,2));  
                std::cout << "laserDirectDistance: " << DirectDistance2 << std::endl;

            
            }
            // else if (i_end < i_start){
            //     std::cout << "TURTLEBOT FOUND METHOD 2 ---" << std::endl;

            //     // turtlebot_position = pt1;
            //     pointVector.push_back(pt1);
            //     double DirectDistance3 = sqrt(std::pow(pt1.x,2) + std::pow(pt1.y,2));  
            //     std::cout << "laserDirectDistance: " << DirectDistance3 << std::endl;
            // }
        }
    }

    std::cout << "new scan===================================================-" << std::endl;
    if(!pointVector.empty()){
        turtlebot_position = pointVector.at(0);    
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
geometry_msgs::Point Sensorprocessing::findTurtlebotworld(){
    geometry_msgs::Point turtlebot_position; //float for the average range of that segments and int is the center point of said segments
    std::vector<geometry_msgs::Point> pointVector;

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
                
                currentRange = Image_data.laserScan.ranges.at(i);
                prevRange = Image_data.laserScan.ranges.at(i - 1);
                Distance_Between_scanpoint = fabs(currentRange - prevRange);
                i++;
            } 
            
            int i_end = i-1;
            int i_center = i_start+(i_end-i_start)/2;

            geometry_msgs::Point pt1 =polarToCart(i_start);
            geometry_msgs::Point pt2 =polarToCart(i_end);

            double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
               
            if (distance >= Turtlebot_min && distance < Turtlebot_max){
               
                geometry_msgs::Point segment_center;
                segment_center.x = ((pt1.x + pt2.x) / 2);
                segment_center.y = ((pt1.y + pt2.y) / 2);                
                turtlebot_position = segment_center;
                pointVector.insert(pointVector.begin(), segment_center);
            }
            else if (i_end < i_start){
                pointVector.push_back(pt1);
            }
        }
    }
    if(!pointVector.empty()){
        turtlebot_position = pointVector.at(0);    
    }
return turtlebot_position;

}

/**
 * @brief Find laser points from laser scan data.
 *
 * This function processes laser scan data and extracts the valid laser points
 * in the form of 2D Cartesian coordinates. It iterates through the laser scan
 * data, checks for valid (non-infinite) readings, converts polar coordinates to
 * Cartesian coordinates, and stores the points in a vector.
 *
 * @return A vector of 2D points representing the valid laser scan data.
 */
std::vector<geometry_msgs::Point> Sensorprocessing::findAllLaserPoints(){
    std::vector<geometry_msgs::Point> pointVector;

    for (int i = 1; i < Image_data.laserScan.ranges.size(); i++) { // for all readings
        if(!std::isinf(Image_data.laserScan.ranges.at(i))){ // if the number isn't infinity
            geometry_msgs::Point pt1 =polarToCart(i);
            pointVector.push_back(pt1);
        }
    }
    return pointVector;
}
