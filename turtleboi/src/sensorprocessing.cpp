#include "sensorprocessing.h"


using namespace std;

Sensorprocessing::Sensorprocessing(RobotData New_Data){   
    Image_data = New_Data;
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
    std::vector<std::pair<geometry_msgs::Point, int>> segments; //float for the average range of that segments and int is the center point of said segments

    for (int i = 1; i < laserScan_.ranges.size(); i++) { // for all readings
        float currentRange = laserScan_.ranges.at(i);
        float prevRange = laserScan_.ranges.at(i - 1);
        float distance = fabs(currentRange - prevRange);





        if(!std::isinf(laserScan_.ranges.at(i))){ // if the number isn't infinity
            int i_start = i;
          
            while(!std::isinf(laserScan_.ranges.at(i))&& distance <= 0.3){
                if (i == laserScan_.ranges.size()-1){
                    
                    break;
                }
                i++;
                currentRange = laserScan_.ranges.at(i);
                prevRange = laserScan_.ranges.at(i - 1);
                distance = fabs(currentRange - prevRange);
            } 
            
            int i_end = i-1;
            int i_center = i_start+(i_end-i_start)/2;

            geometry_msgs::Point pt1 =polarToCart(i_start);
            geometry_msgs::Point pt2 =polarToCart(i_end);

            double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);

            if (distance < 0.5){

                geometry_msgs::Point Cone_center;

                Cone_center.x = ((pt1.x + pt2.x) / 2);
                Cone_center.y = ((pt1.y + pt2.y) / 2);
            
               
                segments.push_back(make_pair(Cone_center, i_center));
            }
        }
    }

    return segments;

}

    std::vector<geometry_msgs::Point> LaserProcessing::GetCones(){
    std::vector<geometry_msgs::Point> cones;
    std::vector<std::pair<geometry_msgs::Point, int>> segments = findAllCones(); 

    for (auto element : segments){
        cones.push_back(element.first);
    }
}





geometry_msgs::Point Sensorprocessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}


