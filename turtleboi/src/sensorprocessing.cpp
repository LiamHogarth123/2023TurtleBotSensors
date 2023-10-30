#include "sensorprocessing.h"


using namespace std;

Sensorprocessing::Sensorprocessing(){   
    
    Turtlebot_min = 0.02;
    Turtlebot_max = 0.08;
}

void Sensorprocessing::Newdata(RobotData temp_data){
    Image_data = temp_data;
    
}

void Sensorprocessing::PrintLaserSpec(){
    
    std::cout << "min" << std::endl;
    std::cout << Image_data.laserScan.angle_min << std::endl;
    std::cout << "max" << std::endl;
    std::cout << Image_data.laserScan.angle_max << std::endl;
    std::cout << "increment" << std::endl;
    std::cout << Image_data.laserScan.angle_increment << std::endl;
}



geometry_msgs::Point Sensorprocessing::findTurtlebot(){
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

void Sensorprocessing::findsegments(){
   geometry_msgs::Point turtlebot_position; //float for the average range of that segments and int is the center point of said segments


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


            // std::cout << "I start" << std::endl;
            // std::cout << i_start << std::endl;
            // std::cout << i_end << std::endl;
            
            // std::cout << "I end" << std::endl;
        
            // std::cout << "laserscan points" << std::endl;
            // std::cout << pt1.x << std::endl;
            // std::cout << pt1.y << std::endl;
            // std::cout << pt2.x << std::endl;
            // std::cout << pt2.y << std::endl;
            // std::cout << "laserscan points end" << std::endl;

            double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);

                std::cout << "distance between scan" << std::endl;
                std::cout << distance << std::endl;
            
            if (distance >= Turtlebot_min && distance < Turtlebot_max){
                
                std::cout << "TURTLEBOT FOUND METHOD 1 ---" << std::endl;

                geometry_msgs::Point segment_center;

                segment_center.x = ((pt1.x + pt2.x) / 2);
                segment_center.y = ((pt1.y + pt2.y) / 2);
                
                turtlebot_position = segment_center;
            
            
            }
            else if (i_end < i_start){
                std::cout << "TURTLEBOT FOUND METHOD 2 ---" << std::endl;
                turtlebot_position = pt1;
            }
        }
    }

    

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
