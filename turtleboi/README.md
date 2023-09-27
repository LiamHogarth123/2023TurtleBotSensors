TURTLEBOI FOLLOWING - Discussion
=========================

This is the code I have completed so far. So far the turtlebot is moving and is functional.

### Compiling

Before you get started, make sure you do the following:

* Check out the latest code from the repository
* Link the `turtleboi` folder to your catkin workspace, (ie if your path is <YOURGIT>/tutorial/skeleton/a3_skeleton then execute:
```bash
cd ~/catkin_ws/src
ln -s <YOURGIT>/<Reporsitoy name>/turtleboi
```

* Compile packages using `catkin_make` 
```bash
cd ~/catkin_ws
catkin_make
```


### Execution

```bash
roscore
roslaunch turtlebot3_fake turtlebot3_fake.launch
rosrun turtleboi turtleboi_method
```

The code:

* Subscribes to some topics / Advertises some topics / Advertises a service
* Has a seperate thread of execution where it sends steering 1.0 rad, throttle 0.1 every 0.2s 

### **Code Structure**

This code structure has 3 sections. Similar to PFMS we have has cpp file called method that was called sample in PFMS. Remeber the main method creates a new thread running within method.

The method file handles all ros communication such as publishing and subscribing. This method also handle all communciation with the two libaries to do with movenment and machine vision. 

The method file subscribes topics of odom (odometry data), Camera RGB, Camera Depth Data, Lid Sensor.

This method file will publish to cmd_vel which sends a velocity to the turtle bot in the formatt of geometry_msg::Twist which is object. For example and object of type twist inclues object.linear.x,y,z and object.angular.x,y,z. 

This will be the major challenge of movenemnt velocity in tranforming the data of two points into these values.

Additionly I might added another publisher to added goal markers to the rviz


The method file communciates with the sensorprocessing file by providing it all of the sensor data in a specisted object called RobotData. The method file combines all of the subscribed data using mutex and call backs into this structure and sends it to the snesor processing libary.

for example an object called data of type robotdata will be able to do data.lidata, data.RGB, or data.imageDepth

The goal is to have a major calcuation function that will return a point in x,y for the movenment libary.

The method file will send this point from sensory data processing and the current position from odom to the movenment library. This libraying will have function such as calculate the trajectory and distance to goal. The same sort stuff we did in PFMS but not as detailed and different with the movenment type


### Documentation

In source documentation is a must, we will examine all code submitted for supporting documentation. You also need the dox file (mainpage.dox) to indicate how the code will run/behave. You need to modiy the mainpage.dox included which does not have any specific documentation.

In ROS, doxygen documentation in generated using the `rosdoc_lite` tool. If you do not have the tool you can install it via `sudo apt-get install ros-noetic-rosdoc-lite` (replace noetic with melodic if on 18.04)

To generate the documentation'

```bash
cd ~/catkin_ws/src/a3_skeleton
rosdoc_lite .
```

You will find the documentation inside doc folder.

```bash
firefox ~/catkin_ws/src/a3_skeleton/doc/html/index.html
```






[services_masterclass]: starter/services_masterclass
[utest.cpp]: starter/services_masterclass/test/utest.cpp
[GridProcessing]: starter/services_masterclass/grid_processing.h
[quiz5a]: ../../quizzes/quiz5/a
[pfms_support]: ../../skeleton/pfms_support
