TURTLEBOI Information
=========================

### Compiling

Before you get started, make sure the following is completed:

* Ensure that your code is saved and you have pulled from the repository
* Compile your code by pressing Build in VScode
* Compile packages using `catkin_make` 
```bash
cd ~/catkin_ws
catkin_make
```


### Execution

```bash
roscore
roslaunch turtlemulti turtleMulti.launch
rosrun turtleboi turtleboi_method
```

The code:

* Subscribes to some topics / Advertises some topics / Advertises a service
* Has a seperate thread of execution where it sends steering 1.0 rad, throttle 0.1 every 0.2s 

### **Code Structure**

This code structure has 4 main sections. We have has cpp file called method that subscribes and publishes data through ROS to the simulator, the main method creates a new thread running within method, SensorProcessing filters and processes scan data, and the Movement class does all the trajectory calculations for movement.

![Blank diagram (3)](https://github.com/LiamHogarth123/2023TurtleBotSensors/assets/126121211/82dd4d19-e188-4825-aee1-d90663f161f6)


### **Method Class File (main parent file)**

The method file handles all ros communication such as publishing and subscribing. This method also handle all communciation with the two libaries to do with movenment and processing sensor data

The method file subscribes topics of odom (odometry data), Camera RGB, Camera Depth Data, Lida Sensor.

This method file will publish to cmd_vel which sends a velocity to the turtle bot in the formatt of geometry_msg::Twist which is object. For example and object of type twist inclues object.linear.x,y,z and object.angular.x,y,z. 



### **SensorProcesing Class File**
The method file communciates with the Movement Class file by providing it all of the sensor data in a specisted object called RobotData. The method file combines all of the subscribed data using mutex and callbacks into this structure and sends it to the Movement libary as a x,y goal location.

for example an object called data of type robotdata will be able to do data.lidata, data.RGB, or data.imageDepth

```c++
struct RobotData 
    sensor_msgs::Image rgbImage;    // Camera RGB image data
    sensor_msgs::Image depthImage;  // Camera depth image data
    sensor_msgs::LaserScan laserScan;  // Laser scan data
```


### **Movement Class File**

The Movement Class recieves odom data and goal positions from the Method and SensorProcessing classes to be processes into movement velocities for the turtlebots. This library will have functions such as calculate the trajectory velocities to reach the target goal positions for each turtlebot. It also manages slowing and stopping when it reaches the goals. 
