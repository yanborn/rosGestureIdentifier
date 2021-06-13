#include "gestureIdentifier.h"

#include <ros/ros.h>

int
main(int argc, char **argv)
{
  ROS_INFO("ROS Gesture GUI started!");

  ros::init(argc, argv, "gestureIdentifier");
  gestureIdentifier identifier;

  identifier.startIdentifier();

  return 0;
}
