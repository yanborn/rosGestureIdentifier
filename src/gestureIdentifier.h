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

  static bool leftDropdownIsHighlighted;
  static bool leftDropdownIsClicked;
  static bool rightDropdownIsHighlighted;
  static bool rightDropdownIsClicked;
  static bool sliderIsHighlighted;
  static bool sliderIsClicked;

  bool runTfLoop();

  void checkHighlight();
  void checkClick();
  void checkUpDown();
};

#endif // GESTUREIDENEITIFER_H
