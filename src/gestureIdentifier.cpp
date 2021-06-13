#include "gestureIdentifier.h"

#include <std_msgs/String.h>

gestureIdentifier::gestureIdentifier()
{
}

gestureIdentifier::~gestureIdentifier() = default;

bool
gestureIdentifier::startIdentifier()
{
  ros::NodeHandle nh;
  gesturePublisher = nh.advertise<std_msgs::String>("gestureGui", 1000);

  return runTfLoop();
}

bool
gestureIdentifier::runTfLoop()
{
  ros::Rate loopRate(10.0);

  while(ros::ok())
  {
    tf::StampedTransform transform;
    try {
      tfListener.lookupTransform("/openni_depth_frame", "/left_hand_1", ros::Time(0), transform);
    } catch (tf::TransformException &e) {
      ROS_ERROR_STREAM("Error transforming tf.");
    }

    loopRate.sleep();
  }

  return 0;
}
