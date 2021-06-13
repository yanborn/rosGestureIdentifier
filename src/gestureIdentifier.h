#ifndef GESTUREIDENEITIFER_H
#define GESTUREIDENEITIFER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

class gestureIdentifier
{
public:
  gestureIdentifier();
  ~gestureIdentifier();

  bool startIdentifier();

private:
  ros::Publisher gesturePublisher;
  tf::TransformListener tfListener;

  bool runTfLoop();
};

#endif // GESTUREIDENEITIFER_H
