/**
 * @file main.cpp
 * @brief Main file to run the gesture identifier
 *
 * @author Yannick Volkenborn
 * @author Jan Dominik Wisker
 * @author Saidjamol Akbarov
 */

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
