#include "sensorprocessing.h"


using namespace std;

Sensorprocessing::Sensorprocessing(){   
    Turtlebot_width = 0.2;
}

void Sensorprocessing::Newdata(RobotData temp_data){
    Image_data = temp_data;
}



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


geometry_msgs::Point Sensorprocessing::polarToCart(unsigned int index)
{
    
    float angle = Image_data.laserScan.angle_min + Image_data.laserScan.angle_increment*index;// + angle_range/2;
    float range = Image_data.laserScan.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}


