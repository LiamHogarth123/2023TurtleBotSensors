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
    return goal_location;
}

