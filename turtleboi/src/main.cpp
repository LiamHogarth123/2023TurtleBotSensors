
#include "ros/ros.h"
#include "method.h"
#include <thread>


int main(int argc, char **argv){


  ros::init(argc, argv, "turtleboi");

  ros::NodeHandle nh;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on the function desired
   */
  std::shared_ptr<Method> method(new Method(nh));
  std::thread t(&Method::seperateThread, method);

  // std::thread t(&Sample::control,sample);

  ros::spin();

  ros::shutdown();

  t.join();

  return 0;
}

