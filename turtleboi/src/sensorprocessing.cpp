#include "sensorprocessing.h"


using namespace std;

Sensorprocessing::Sensorprocessing(){   
    Turtlebot_width = 210;
}

void Sensorprocessing::Newdata(RobotData temp_data){
    Image_data = temp_data;
}


geometry_msgs::Point Sensorprocessing::CalculateMidPoint(){
    geometry_msgs::Point goal_location;

    //do some image processing
    Image_data.laserScan;
    //testing 
    goal_location.x = 5;
    goal_location.y = 5;
    return goal_location;
}

geometry_msgs::Point Sensorprocessing::findTurtlebot(){
    geometry_msgs::Point segments; //float for the average range of that segments and int is the center point of said segments

        for (int i = 1; i < Image_data.laserScan.ranges.size(); i++) { // for all readings
            // float currentRange = Image_data.laserScan.ranges.at(i);
            // float prevRange = Image_data.laserScan.ranges.at(i - 1);
            // float range = fabs(currentRange - prevRange);


            if(!std::isinf(Image_data.laserScan.ranges.at(i))){ // if the number isn't infinity
                int i_start = i;
            
                while(!std::isinf(Image_data.laserScan.ranges.at(i))){
                    if (i == Image_data.laserScan.ranges.size()-1){
                        
                        break;
                    }
                    i++;
                    // currentRange = Image_data.laserScan.ranges.at(i);
                    // prevRange = Image_data.laserScan.ranges.at(i - 1);
                    // range = fabs(currentRange - prevRange);
                } 
                
                int i_end = i-1;
                int i_center = i_start+(i_end-i_start)/2;

                geometry_msgs::Point pt1 =polarToCart(i_start);
                geometry_msgs::Point pt2 =polarToCart(i_end);

                double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);

                if (distance < (Turtlebot_width+0.1) && distance > (Turtlebot_width - 0.1)){

                    geometry_msgs::Point Cone_center;

                    Cone_center.x = ((pt1.x + pt2.x) / 2);
                    Cone_center.y = ((pt1.y + pt2.y) / 2);
                    
                    segments = Cone_center;
                
                
                }
            }
        }

    return segments;

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


