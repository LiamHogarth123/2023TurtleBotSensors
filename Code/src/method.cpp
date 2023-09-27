#include "sample.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include <tf/LinearMath/Quaternion.h>

#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

using std::cout;
using std::endl;

Sample::Sample(ros::NodeHandle nh) :
  nh_(nh),
  laserProcessingPtr_(nullptr)
{
//Ros construction
    //Subscribing to odometry UGV
    sub1_ = nh_.subscribe("ugv_odom", 1000, &Sample::odomCallback,this);

    sub2_ = nh_.subscribe("orange/laser/scan", 10, &Sample::laserCallback,this);

    sub3_ = nh_.subscribe("/orange/goals", 1000, &Sample::goalsCallback, this);

    
    //Publishing markers

    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);

    Cone_pub = nh_.advertise<geometry_msgs::PoseArray>("orange/cones", 1000,true);

    pubBrakeCmd_ = nh_.advertise<std_msgs::Float64>("/orange/brake_cmd",100,false);
    pubSteering = nh_.advertise<std_msgs::Float64>("orange/steering_cmd",100,false);
    pubThrottle = nh_.advertise<std_msgs::Float64>("orange/throttle_cmd",100,false);

    //Allowing an incoming service on /request_goal (you need to change name depending on project)
    service_ = nh_.advertiseService("/orange/mission", &Sample::request,this);

    

    //Ackerman contruction
    //type = pfms::PlatformType::ACKERMAN;
    steering_ratio = 17.3;
    lock_to_lock_Revs = 3.2;
    max_steering_Angle = (M_PI * lock_to_lock_Revs / steering_ratio);
    track = 1.638;
    wheel_radius = 0.36;
    wheel_base = 2.65;
    max_brake_torque = 8000;
    default_throttle = 0.1; //(top speed 2.91)
    average_velocity = 2.91;
    run = false;

    FoundCones.clear();

    
};

// We delete anything that needs removing here specifically
Sample::~Sample(){

    if(laserProcessingPtr_ != nullptr){
        delete laserProcessingPtr_;
    }
}


//Call backs/////////////////////////////////////////////////////////////////////////////
// A callback for odometry
void Sample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    std::unique_lock<std::mutex> lck1 (odoMtx_);
    CarPosition = msg->pose.pose;
    speed = std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x + msg->twist.twist.linear.y * msg->twist.twist.linear.y);
    // std::unique_lock<std::mutex> lck1 (robotPoseMtx_);
    // robotPose_ = pose; // We store a copy of the pose in robotPose_
}





// A callback for odometry
void Sample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    std::unique_lock<std::mutex> lck2 (laserDataMtx_);
    // std::unique_lock<std::mutex> lck(laserDataMtx_);
    laserData_ = *msg; // We store a copy of the LaserScan in laserData_
}

void Sample::goalsCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    // std::unique_lock<std::mutex> lck1 (odoMtx_);
    // std::unique_lock<std::mutex> lck2 (laserDataMtx_);
    std::unique_lock<std::mutex> lck3 (goalMtx_);
    goals = *msg;
    std::vector<geometry_msgs::Pose> current_Goals = goals.poses;
    if (!current_Goals.empty()) {
        goals_set = true;
    }
    

}


//Main running thread /////////////////////////////////////////////////////////////////////////////
void Sample::seperateThread() {
   
    //! THINK : What rate shouls we run this at? What does this limiter do?
    ros::Rate rate_limiter(5.0);
    while (ros::ok()) {
     
        //sendCmd(0,0, 100);
         std::this_thread::sleep_for(std::chrono::milliseconds(50));//Small delay to ensure message sent
         std::this_thread::sleep_for(std::chrono::milliseconds(50));//Small delay to ensure message sent
        std::this_thread::sleep_for(std::chrono::milliseconds(50));//Small delay to ensure message sent
        
       
        control();
        rate_limiter.sleep();

    }
}


///Service requests /////////////////////////////////////////////////////////////////////////
bool Sample::request(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res)
{
    ROS_INFO_STREAM("Requested:" << req.data);

    if (req.data){
         if (goalSet_){
            status_ = Ackerman::RUNNING;
            executeGoals_ = true;
        }
        else {
            status_ = Ackerman::IDLE;
            executeGoals_ = true;
        }
 
        res.success = true;  

    } 
    else {
        //don't want to execute any more goals
        executeGoals_ = true;
        status_ = Ackerman::IDLE;
    }

    //We can reply in the two field of the return value
    res.success = true;

    // return message for the current status
    switch(status_){
        case Ackerman::IDLE:
            res.message = "IDLE";
            break;
        case Ackerman::RUNNING:
            res.message = "RUNNING";
            break;
        default:
            res.message = "UNKNOWN";
            break;
    }
    return true; //We return true to indicate the service call sucseeded (your responce should indicate a value)
}

bool Sample::requestadvance(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res){
            
    ROS_INFO_STREAM("Requested:" << req.data);

    if (req.data){
        basic = true;
    }
    else {
        basic = true;
    }
    return true;
}



//ACKERMAN//////////////////////////////////////////////////////////////////////////////////

bool Sample::checkOriginToDestination(geometry_msgs::Pose origin,geometry_msgs::Point goal, double &distance, double &steering_anlge_, geometry_msgs::Pose estimatedGoalPose){
    //estimatedGoalPose.yaw = 0;
    nav_msgs::Odometry origin1;
  

    //this line finds the maxium possible steering angle to check wheather the calculated steering angle is within range.
    double max_steering_Angle = (M_PI*lock_to_lock_Revs/steering_ratio);
    double esimtadyaw = 0;
   

    double x_diff = goal.x - origin.position.x;
    double y_diff = goal.y - origin.position.y;
    double Direct_diff = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    Heading_diff = atan2(y_diff,x_diff)-(tf::getYaw(origin.orientation));
    double angle = atan((2*wheel_base*sin(Heading_diff))/Direct_diff);    

    
    double radius = wheel_base/tan(angle);
    steering_anlge_ = angle * steering_ratio;
    
       
    // //checks if the steering angle is within it's maxium steering angle.
    if (abs(angle) > max_steering_Angle){
        time = -1;
        distance = -1;
        return false;
    }
    else {
        esimtadyaw = 2 * Heading_diff + tf::getYaw(origin.orientation);
        //estimatedGoalPose.position = {goal.x, goal.y, 0};

        if (esimtadyaw > M_PI) {
            esimtadyaw -= 2 * M_PI;
        }
        if (esimtadyaw < -M_PI) {
            esimtadyaw += 2 * M_PI;
        }
    
        distance = abs(radius*abs(2*Heading_diff));        
        time = average_velocity/default_throttle + (distance - pow(2.91,2)/(2*default_throttle));   
        
        tf::Quaternion finalOrientation;
        finalOrientation.setRPY(0.0, 0.0, esimtadyaw);


       

        ///////STEERING ANGLE ADJUSTMENT DUE TO SIMULATION FAULT
        
      if ( steering_anlge_ < 0.05 &&  steering_anlge_ > 0 ){
            steering_anlge_ = 0.05;
        }
        else if ( steering_anlge_ > -0.05 &&  steering_anlge_ < 0){
            steering_anlge_ = 0.05;
        }

       
        geometry_msgs::Quaternion estimatedQuaternion;
        tf::quaternionTFToMsg(finalOrientation, estimatedQuaternion);

        estimatedGoalPose.orientation = estimatedQuaternion;
       
    
        
        return true;
        }
    }

void Sample::sendCmd(double brake, double steering, double throttle) {
    // std::cout << "CMD is sending??" << std::endl;
    std_msgs::Float64 brake_;
    brake_.data = brake;
    std_msgs::Float64 Steering_;
    Steering_.data = steering;
    std_msgs::Float64 throttle_;
    throttle_.data = throttle;

    pubBrakeCmd_.publish(brake_);
    pubSteering.publish(Steering_);
    pubThrottle.publish(throttle_);
}

bool Sample::reachGoals() {
    bool goal_reached = false;
    double steering_angle_boi;
    double distance;

    
    
    while (true){
        
        LaserProcessing laserProcessing(laserData_);
        bool truck = laserProcessing.isTruck(track, wheel_base);

        std::unique_lock<std::mutex> lck1 (odoMtx_);
        geometry_msgs::Pose Current_Position = CarPosition;
        double current_speed = speed;
        lck1.unlock();
        
        
        bool x = checkOriginToDestination(Current_Position, goal_.position, distance, steering_angle_boi, estimatedGoalPose);
        
     
        
        if ((distance - Caroffset) < (tolerance || truck)){ //||truck
            
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));//Small delay to ensure message sent
       
        
        ///////SPEED CONTROL////////////////      
        if (current_speed >= average_velocity){
            
            sendCmd(0, steering_angle_boi,  0);
        }
        else if (distance <= 2){
            // std::cout << "working" << std::endl;
            sendCmd(0, steering_angle_boi,  0.4);
        }
        else {
            sendCmd(0, steering_angle_boi,  0.1);
        }
        
  
    }
  
    
 
    return true;

}



  Ackerman::PlatformStatus Sample::getStatus(){
    return status_;
  }


//CONTROLLER///////////////////////////////////////////////////////////////////////////////





void Sample::control(){

    std::vector<geometry_msgs::Pose> current_Goals = goals.poses;

    // only move the quadcopter if the goal is set and the execute goal flag is set.
    while(!goalSet_ || !executeGoals_ || current_Goals.empty()){
        sendCmd(0, 0, 0);
        //it will do it for the exact amount of time needed to run at 1Hz
        // rate_limiter.sleep();
        //This will make the code return back to beginig of while loop
      current_Goals = goals.poses;
    }
    
    current_Goals = goals.poses;
    
    LaserProcessing laserProcessing(laserData_);
    DataProcessing DataProcessor;
    std::vector<geometry_msgs::Point> midPoint_Storage;

    if (basic ){ //if basic
        Caroffset = -0.3;

     

        if (AllGoalReached == false){
            
            for (int i = 0; i < current_Goals.size(); i++){
                goal_=current_Goals.at(i);
            

                //updating odom with mutex
                std::unique_lock<std::mutex> lck1 (odoMtx_);
                geometry_msgs:: Pose Current_place = CarPosition;  
                lck1.unlock();

                //updating laserdata with muex
                sensor_msgs::LaserScan temp;
                std::unique_lock<std::mutex> lck2 (laserDataMtx_); 
                temp = laserData_;
                lck2.unlock();
                laserProcessing.newScan(temp);


                //Laser data processing and visualisation of mid point
                geometry_msgs::Point Current_mid_point = laserProcessing.detectMidPointBasic();
                std::vector<geometry_msgs::Point> cones = laserProcessing.GetCones();
                std::vector<geometry_msgs::Point> AllMidPoints = laserProcessing.detectMidPointAdvancetest();
                
                //adjusting data
                cones = DataProcessor.adjustLaserData(cones, Current_place);
                geometry_msgs::Point Current_mid_point_World = DataProcessor.adjustLaserData(Current_mid_point, Current_place);
                std::vector<geometry_msgs::Point> temps = DataProcessor.adjustLaserData(AllMidPoints, Current_place);
                
                for (auto element : temps){
                    midPoint_Storage.push_back(element);
                }

                for (const auto& element : cones) {
                bool newCone = true;
                    for (const auto& cone : FoundCones) {
                        float distance = std::sqrt(std::pow(element.x - cone.x, 2) + std::pow(element.y - cone.y, 2));
                        if (distance < 1) {
                            newCone = false;
                            break;
                        }
                    }
                    if (newCone) {
                        FoundCones.push_back(element);
                    }
                }


                visualization_msgs::MarkerArray marker;

                marker = DataProcessor.visualiseRoadCenter(Current_mid_point_World, marker);
        
                marker = DataProcessor.visualiseCone(FoundCones, marker);
                
                pub_.publish(marker);

         
        
                geometry_msgs::PoseArray Cones;
                Cones.header.frame_id = "world";
                Cones.header.stamp = ros::Time::now();
                Cones.header.seq = ct;
                ct ++;
                geometry_msgs::Pose current_cone;
                for (auto element : FoundCones){
                    current_cone.position = element;
                    current_cone.orientation.x = 0.0;
                    current_cone.orientation.y = 0.0;
                    current_cone.orientation.z = 0.0;
                    current_cone.orientation.w = 1.0;
                    Cones.poses.push_back(current_cone);
                }
                Cone_pub.publish(Cones);
                            
                //Laser data Processing and visualisation of cones
             
         

                //if goal is within the cones
                          
                if (DataProcessor.goalWithinCone(goal_.position, temps) || DataProcessor.goalWithinCone(goal_.position, Current_mid_point_World)){
                    
                    if (checkOriginToDestination(Current_place, goal_.position, distance, time, estimatedGoalPose)){                
                        bool reachable = reachGoals();
                        }
                    }
                else {
                    sendCmd(max_brake_torque,0,0);
                    break;
                }
         
            }
        }
        sendCmd(max_brake_torque,0,0);
        AllGoalReached = true;

    }

    //////ADVANCE///////////////////////////////////////////////////////////////////////////////
    else if (!basic){ //if advance
        
        
        Caroffset = -0.3;

        
        if (AllGoalReached == false){
            
            for (int i = 0; i < current_Goals.size(); i++){
                goal_=current_Goals.at(i);
            

                //updating odom with mutex
                std::unique_lock<std::mutex> lck1 (odoMtx_);
                geometry_msgs:: Pose Current_place = CarPosition;  
                lck1.unlock();

                //updating laserdata with muex
                sensor_msgs::LaserScan temp;
                std::unique_lock<std::mutex> lck2 (laserDataMtx_); 
                temp = laserData_;
                lck2.unlock();
                laserProcessing.newScan(temp);


                //Laser data processing and visualisation of mid point
                geometry_msgs::Point Current_mid_point = laserProcessing.detectMidPointBasic();
                std::vector<geometry_msgs::Point> cones = laserProcessing.GetCones();
                std::vector<geometry_msgs::Point> AllMidPoints = laserProcessing.detectMidPointAdvancetest();
                
                //adjusting data
                cones = DataProcessor.adjustLaserData(cones, Current_place);
                geometry_msgs::Point Current_mid_point_World = DataProcessor.adjustLaserData(Current_mid_point, Current_place);
                std::vector<geometry_msgs::Point> temps = DataProcessor.adjustLaserData(AllMidPoints, Current_place);
                
                for (auto element : temps){
                    midPoint_Storage.push_back(element);
                }

                for (const auto& element : cones) {
                bool newCone = true;
                    for (const auto& cone : FoundCones) {
                        float distance = std::sqrt(std::pow(element.x - cone.x, 2) + std::pow(element.y - cone.y, 2));
                        if (distance < 1) {
                            newCone = false;
                            break;
                        }
                    }
                    if (newCone) {
                        FoundCones.push_back(element);
                    }
                }


                visualization_msgs::MarkerArray marker;

                marker = DataProcessor.visualiseRoadCenter(Current_mid_point_World, marker);
        
                marker = DataProcessor.visualiseCone(FoundCones, marker);
                
                pub_.publish(marker);

         
        
                geometry_msgs::PoseArray Cones;
                Cones.header.frame_id = "world";
                Cones.header.stamp = ros::Time::now();
                Cones.header.seq = ct;
                ct ++;
                geometry_msgs::Pose current_cone;
                for (auto element : FoundCones){
                    current_cone.position = element;
                    current_cone.orientation.x = 0.0;
                    current_cone.orientation.y = 0.0;
                    current_cone.orientation.z = 0.0;
                    current_cone.orientation.w = 1.0;
                    Cones.poses.push_back(current_cone);
                }
                Cone_pub.publish(Cones);
                            
                //Laser data Processing and visualisation of cones
             
         

                //if goal is within the cones
                         
                if (DataProcessor.goalWithinCone(goal_.position, temps) || DataProcessor.goalWithinCone(goal_.position, Current_mid_point_World)){
                    
                    if (checkOriginToDestination(Current_place, goal_.position, distance, time, estimatedGoalPose)){                
                        bool reachable = reachGoals();
                        }
                    }
                else {
                    sendCmd(max_brake_torque,0,0);
                    break;
                }
         
            }
        }

        Caroffset = 2;


        std::this_thread::sleep_for(std::chrono::milliseconds(500));//Small delay to ensure message sent




        while (true){
            
            //laser data mutex storage
            sensor_msgs::LaserScan temp;
            
            std::unique_lock<std::mutex> lck2 (laserDataMtx_); 
            temp = laserData_;
            lck2.unlock();
            
            laserProcessing.newScan(temp);
           
            
        
            //LaserProcessing functions
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            geometry_msgs::Point Current_mid_point = laserProcessing.detectMidPointAdvance();
            std::vector< geometry_msgs::Point> midpoints = laserProcessing.detectMidPointAdvancetest();
            std::vector<geometry_msgs::Point> cones = laserProcessing.GetCones();

            //check if there is no more cones
            if (Current_mid_point.x == 0 && Current_mid_point.y == 0){ //add a another check to see if there is still visible cones
                std::this_thread::sleep_for(std::chrono::milliseconds(50));//Small delay to ensure message sent
                //waits to see if the car difts around a corner and finds more cones
                
                std::unique_lock<std::mutex> lck2 (laserDataMtx_); 
                temp = laserData_;
                lck2.unlock();
            
                laserProcessing.newScan(temp);
                Current_mid_point = laserProcessing.detectMidPointAdvance();
           


                if(Current_mid_point.x == 0 && Current_mid_point.y == 0){
                    sendCmd(max_brake_torque,0,0);
                    break;
                }
            }

            //Odom Mutex locking data gather
            std::unique_lock<std::mutex> lck1 (odoMtx_);
    
            cones = DataProcessor.adjustLaserData(cones, CarPosition);   
            Current_mid_point = DataProcessor.adjustLaserData(Current_mid_point, CarPosition);
            midpoints = DataProcessor.adjustLaserData(midpoints, CarPosition);   
            lck1.unlock();

            //Assign internal stored variable of current goal
            goal_.position = Current_mid_point;
            
       
            //makes an array of visualisation     
            visualization_msgs::MarkerArray marker;
            // for (auto elenet: midpoints){
            //     marker = visualiseCone(midpoints, marker);
            // }
            marker = DataProcessor.visualiseRoadCenter(goal_.position, marker);
            

            marker = DataProcessor.visualiseCone(cones, marker);

            pub_.publish(marker);
            

           
            reachGoals();

            
           Caroffset = 2;

        }

    }

}