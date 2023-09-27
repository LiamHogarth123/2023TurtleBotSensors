#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan),SecondMidPoint{} 
{   
   
}


geometry_msgs::Point LaserProcessing::detectMidPointBasic() {

    std::vector< geometry_msgs::Point> midpoints;
    geometry_msgs::Point MidPoint_;
    geometry_msgs::Pose midPoint;
    geometry_msgs::Pose tempSecondmidPoint;
    std::vector<std::pair<geometry_msgs::Point, int>> cones = findAllCones();
    float closestdistance = std::numeric_limits<float>::max();
    
    //Small delay to ensure message sent

    

    std::sort(cones.begin(), cones.end(), [this](const std::pair<geometry_msgs::Point, int>& a, const std::pair<geometry_msgs::Point, int>& b) {
        return laserScan_.ranges.at(a.second) < laserScan_.ranges.at(b.second);
    });


    for (int i = 0; i < cones.size()-1; i++ ){
        geometry_msgs::Point pt1 = cones.at(i).first;
        geometry_msgs::Point pt2 = cones.at(i+1).first;


        
        double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
      

        if (distance > 7.84 && distance < 11){
         
            MidPoint_.x = ((pt1.x + pt2.x) / 2);
           MidPoint_.y = (pt1.y + pt2.y) / 2;
            
            midpoints.push_back(MidPoint_);

            // // for (int j = i; j < cones.size()-1; j++){
            // //     pt1 = cones.at(j).first;
            // //     pt2 = cones.at(j+1).first;

            // //     distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
               


            // //     if (distance > 7 && distance < 10){
            // //         tempSecondmidPoint.position.x = ((pt1.x + pt2.x) / 2);
            // //         tempSecondmidPoint.position.y = (pt1.y + pt2.y) / 2;
            // //     }
            
            // // }
            
            // break;


        }

    }
   
    if (midpoints.empty()){
        midPoint = geometry_msgs::Pose();
    }
    else{
        // if (pow(pow(midpoints.at(0).x,2)+pow(midpoints.at(0).y, 2),0.5) > 6){
        //     midPoint.position = midpoints.at(0);
        // }
        // else {
        
        //     for (int i = 0; i < cones.size()-1; i++ ){
        //         geometry_msgs::Point pt1 = cones.at(i).first;
        //         geometry_msgs::Point pt2 = cones.at(i+1).first;



        //         double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
       


        //          if (distance > 7.4 && distance < 30){
        //             midPoint.position.x = ((pt1.x + pt2.x) / 2);
        //             midPoint.position.y = (pt1.y + pt2.y) / 2;
            
        //             midpoints.push_back(midPoint.position);
        //         }
        //     }
            MidPoint_ = midpoints.at(0);
        // }

    }
    // if (midPoint.position.x == 0 && midPoint.position.y == 0){
    //     midPoint.position = lastMidPoints.at(1);
    //     if (lastMidPoints.at(1).x == 0){
    //         midPoint.position = thirdlastMidPoints.at(2);
    //     }
    //     midPoint.position = lastMidPoints.at(1);
    // }
    // else {
    //     thirdlastMidPoints = lastMidPoints;
    //     lastMidPoints = midpoints;    
    // }
    
    
    return MidPoint_;


}





geometry_msgs::Point LaserProcessing::detectMidPointAdvance(){
    std::vector< geometry_msgs::Point> midpoints;
    geometry_msgs::Point MidPoint_;
    geometry_msgs::Pose midPoint;
    geometry_msgs::Pose tempSecondmidPoint;
    std::vector<std::pair<geometry_msgs::Point, int>> cones = findAllCones();
    float closestdistance = std::numeric_limits<float>::max();
    
    //Small delay to ensure message sent

    

    std::sort(cones.begin(), cones.end(), [this](const std::pair<geometry_msgs::Point, int>& a, const std::pair<geometry_msgs::Point, int>& b) {
        return laserScan_.ranges.at(a.second) < laserScan_.ranges.at(b.second);
    });


    for (int i = 0; i < cones.size()-1; i++ ){
        geometry_msgs::Point pt1 = cones.at(i).first;
        geometry_msgs::Point pt2 = cones.at(i+1).first;


        
        double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
      


        if (distance > 7.8 && distance < 11){
            MidPoint_.x = ((pt1.x + pt2.x) / 2);
           MidPoint_.y = (pt1.y + pt2.y) / 2;
            
            midpoints.push_back(MidPoint_);

            // // for (int j = i; j < cones.size()-1; j++){
            // //     pt1 = cones.at(j).first;
            // //     pt2 = cones.at(j+1).first;

            // //     distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
               


            // //     if (distance > 7 && distance < 10){
            // //         tempSecondmidPoint.position.x = ((pt1.x + pt2.x) / 2);
            // //         tempSecondmidPoint.position.y = (pt1.y + pt2.y) / 2;
            // //     }
            
            // // }
            
            // break;


        }

    }
   
    if (midpoints.empty()){
        midPoint = geometry_msgs::Pose();
    }
    else{
        // if (pow(pow(midpoints.at(0).x,2)+pow(midpoints.at(0).y, 2),0.5) > 6){
        //     midPoint.position = midpoints.at(0);
        // }
        // else {
        
        //     for (int i = 0; i < cones.size()-1; i++ ){
        //         geometry_msgs::Point pt1 = cones.at(i).first;
        //         geometry_msgs::Point pt2 = cones.at(i+1).first;



        //         double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
       


        //          if (distance > 7.4 && distance < 30){
        //             midPoint.position.x = ((pt1.x + pt2.x) / 2);
        //             midPoint.position.y = (pt1.y + pt2.y) / 2;
            
        //             midpoints.push_back(midPoint.position);
        //         }
        //     }
            MidPoint_ = midpoints.at(0);
        // }

    }
    // if (midPoint.position.x == 0 && midPoint.position.y == 0){
    //     midPoint.position = lastMidPoints.at(1);
    //     if (lastMidPoints.at(1).x == 0){
    //         midPoint.position = thirdlastMidPoints.at(2);
    //     }
    //     midPoint.position = lastMidPoints.at(1);
    // }
    // else {
    //     thirdlastMidPoints = lastMidPoints;
    //     lastMidPoints = midpoints;    
    // }
    
    
    return MidPoint_;
}



std::vector< geometry_msgs::Point> LaserProcessing::detectMidPointAdvancetest(){
    std::vector< geometry_msgs::Point> midpoints;
    geometry_msgs::Pose midPoint;
    geometry_msgs::Pose tempSecondmidPoint;
    std::vector<std::pair<geometry_msgs::Point, int>> cones = findAllCones();
    float closestdistance = std::numeric_limits<float>::max();
    
    //Small delay to ensure message sent

    

    std::sort(cones.begin(), cones.end(), [this](const std::pair<geometry_msgs::Point, int>& a, const std::pair<geometry_msgs::Point, int>& b) {
        return laserScan_.ranges.at(a.second) < laserScan_.ranges.at(b.second);
    });


    for (int i = 0; i < cones.size()-1; i++ ){
        geometry_msgs::Point pt1 = cones.at(i).first;
        geometry_msgs::Point pt2 = cones.at(i+1).first;


        
        double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
   

        if (distance > 7.9 && distance < 10){
            midPoint.position.x = ((pt1.x + pt2.x) / 2);
            midPoint.position.y = (pt1.y + pt2.y) / 2;


            
            midpoints.push_back(midPoint.position);

            // // for (int j = i; j < cones.size()-1; j++){
            // //     pt1 = cones.at(j).first;
            // //     pt2 = cones.at(j+1).first;

            // //     distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
               


            // //     if (distance > 7 && distance < 10){
            // //         tempSecondmidPoint.position.x = ((pt1.x + pt2.x) / 2);
            // //         tempSecondmidPoint.position.y = (pt1.y + pt2.y) / 2;
            // //     }
            
            // // }
            
            // break;


        }

    }
   
    if (midpoints.empty()){
        midPoint = geometry_msgs::Pose();
    }
    else{
        // if (pow(pow(midpoints.at(0).x,2)+pow(midpoints.at(0).y, 2),0.5) > 6){
        //     midPoint.position = midpoints.at(0);
        // }
        // else {
        
        //     for (int i = 0; i < cones.size()-1; i++ ){
        //         geometry_msgs::Point pt1 = cones.at(i).first;
        //         geometry_msgs::Point pt2 = cones.at(i+1).first;



        //         double distance = pow(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2),0.5);
       


        //          if (distance > 7.4 && distance < 30){
        //             midPoint.position.x = ((pt1.x + pt2.x) / 2);
        //             midPoint.position.y = (pt1.y + pt2.y) / 2;
            
        //             midpoints.push_back(midPoint.position);
        //         }
        //     }
            midPoint.position = midpoints.at(0);
        // }

    }
    // if (midPoint.position.x == 0 && midPoint.position.y == 0){
    //     midPoint.position = lastMidPoints.at(1);
    //     if (lastMidPoints.at(1).x == 0){
    //         midPoint.position = thirdlastMidPoints.at(2);
    //     }
    //     midPoint.position = lastMidPoints.at(1);
    // }
    // else {
    //     thirdlastMidPoints = lastMidPoints;
    //     lastMidPoints = midpoints;    
    // }

    
    return midpoints;
}



std::vector<std::pair<geometry_msgs::Point, int>> LaserProcessing::findAllCones(){
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
    return cones;

}












bool LaserProcessing::isTruck(double track, double wwheel_base)
{   
    
    // Distance from the center of the car to the edge
    double car_half_width = (track + 1) / 2.0;

    // Distance from the car's center to the front
    double car_front_distance = (wwheel_base +1)/2;  // Specify the distance from the car's center to the front

    // Calculate the angle to the left and right edges of the car
    double angle_to_edge = atan(car_half_width / car_front_distance);

    // Calculate the minimum and maximum angles for the frontal region 
    double front_angle_min = -angle_to_edge;
    double front_angle_max = angle_to_edge;

    // Convert the angle range to indices in the range data
    int front_index_min = static_cast<int>((front_angle_min - laserScan_.angle_min) / laserScan_.angle_increment);
    int front_index_max = static_cast<int>((front_angle_max - laserScan_.angle_max) / laserScan_.angle_increment);

    // Check if there is an object within 5 meters in the frontal region
    bool truck_in_way;
    int segment_size;
    geometry_msgs::Point pt;
    geometry_msgs::Point pt2;
    double distance=0;
    int x;

   
    for (int i = 1; i< laserScan_.ranges.size(); i++){
        double acr_distance=0;
        x= i;
        while (laserScan_.ranges.at(i) < 8.0){
            if (i == laserScan_.ranges.size()-1){
                break;
            }
            else {
                i++;
            }
            
        }

        

        if (i != x){
            
            pt = polarToCart(i);
            pt2 = polarToCart(x);

            distance = pow(pow((pt.x-pt2.x),2)+pow((pt.y-pt2.y),2),0.5);
        }
        
        if (distance > 0.2){
            truck_in_way = true;
            break;
        }
    }

   
    return truck_in_way;
} 
    

void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan){
    laserScan_=laserScan;
}


geometry_msgs::Point LaserProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}

double LaserProcessing::angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}




