#include <ros/ros.h>

#include "EntryClass.hpp"

using namespace std;
using namespace ros;
using namespace tf;

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "single_ladar_and_camera_fusion");
ROS_INFO("ros::init");
  // create filter class
  EntryClass node;
ROS_INFO("EntryClass node");
  ros::spin();

  return 0;
}