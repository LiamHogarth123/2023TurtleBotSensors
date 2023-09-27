
#include "ros/ros.h"
#include "sample.h"
#include <thread>


int main(int argc, char **argv)
{


  ros::init(argc, argv, "a3_skeleton");

  ros::NodeHandle nh;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on the function desired
   */
  std::shared_ptr<Sample> sample(new Sample(nh));
  std::thread t(&Sample::seperateThread,sample);

  // std::thread t(&Sample::control,sample);

  ros::spin();

  ros::shutdown();

  t.join();

  return 0;
}

