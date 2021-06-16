#include "gestureIdentifier.h"

#include <std_msgs/String.h>

bool gestureIdentifier::leftDropdownIsHighlighted{false};
bool gestureIdentifier::leftDropdownIsClicked{false};
bool gestureIdentifier::rightDropdownIsHighlighted{false};
bool gestureIdentifier::rightDropdownIsClicked{false};
bool gestureIdentifier::sliderIsHighlighted{false};
bool gestureIdentifier::sliderIsClicked{false};

gestureIdentifier::gestureIdentifier()
{
}

gestureIdentifier::~gestureIdentifier() = default;

bool
gestureIdentifier::startIdentifier()
{
  ros::NodeHandle nh;
  gesturePublisher = nh.advertise<std_msgs::String>("gestureGui", 1);

  return runTfLoop();
}

bool
gestureIdentifier::runTfLoop()
{
  ros::Rate loopRate(1);

  while(ros::ok())
  {
    if(sliderIsClicked) {
      checkUpDown();
    }
    else if(leftDropdownIsHighlighted || rightDropdownIsHighlighted || sliderIsHighlighted) {
      checkClick();
    }

    if(!leftDropdownIsClicked && !rightDropdownIsClicked && !sliderIsClicked) {
      checkHighlight();
    }

    ros::spinOnce();

    loopRate.sleep();
  }

  return 0;
}

void
gestureIdentifier::checkHighlight()
{
  std_msgs::String msg;
  std::stringstream ss;

  tf::StampedTransform transform;
  try {
    tfListener.lookupTransform("/openni_depth_frame", "/left_hand_1", ros::Time(0), transform);
  } catch (tf::TransformException &e) {
    ROS_ERROR_STREAM("Error transforming left hand tf for highlight check.");
    return;
  }

  if(transform.getOrigin().y() <= -0.2)
  {
    ROS_INFO_STREAM("Highlight on left dropdown detected");
    ss << "leftHighlighted";
    msg.data = ss.str();
    gesturePublisher.publish(msg);
    leftDropdownIsHighlighted = true;
    rightDropdownIsHighlighted = false;
    sliderIsHighlighted = false;
  }
  else if(transform.getOrigin().y() > -0.1 && transform.getOrigin().y() < 0.1)
  {
    ROS_INFO_STREAM("Highlight on right dropdown detected");
    ss << "rightHighlighted";
    msg.data = ss.str();
    gesturePublisher.publish(msg);
    rightDropdownIsHighlighted = true;
    leftDropdownIsHighlighted = false;
    sliderIsHighlighted = false;
  }
  else if(transform.getOrigin().y() > 0.2)
  {
    ROS_INFO_STREAM("Highlight on slider detected");
    ss << "sliderHighlighted";
    msg.data = ss.str();
    gesturePublisher.publish(msg);
    sliderIsHighlighted = true;
    leftDropdownIsHighlighted = false;
    rightDropdownIsHighlighted = false;
  }
  else {
    ROS_INFO_STREAM("No highlighting detected");
  }

}

void
gestureIdentifier::checkClick()
{
  std_msgs::String msg;
  std::stringstream ss;

  tf::StampedTransform transform;
  try {
    tfListener.lookupTransform("/openni_depth_frame", "/right_hand_1", ros::Time(0), transform);
  } catch (tf::TransformException &e) {
    ROS_ERROR_STREAM("Error transforming right hand tf for click check.");
    return;
  }

    if(leftDropdownIsHighlighted && (transform.getOrigin().x() < 0.7)) {
      ROS_INFO_STREAM("Click on left dropdown detected");
      ss << "leftClicked";
      msg.data = ss.str();
      gesturePublisher.publish(msg);
      leftDropdownIsClicked = true;
    }
    else if (rightDropdownIsHighlighted && (transform.getOrigin().x() < 0.7)){
      ROS_INFO_STREAM("Click on right dropdown detected");
      ss << "rightClicked";
      msg.data = ss.str();
      gesturePublisher.publish(msg);
      rightDropdownIsClicked = true;
    }
    else if (sliderIsHighlighted && (transform.getOrigin().x() < 0.7)){
      ROS_INFO_STREAM("Click on slider detected");
      ss << "sliderClicked";
      msg.data = ss.str();
      gesturePublisher.publish(msg);
      sliderIsClicked = true;
    }
    else {
      ROS_INFO_STREAM("No click detected");
    }

}

void
gestureIdentifier::checkUpDown()
{
  std_msgs::String msg;
  std::stringstream ss;

  tf::StampedTransform transform;
  try {
    tfListener.lookupTransform("/openni_depth_frame", "/left_hand_1", ros::Time(0), transform);
  } catch (tf::TransformException &e) {
    ROS_ERROR_STREAM("Error transforming left hand tf for up/down check.");
    return;
  }

    if(sliderIsClicked && (transform.getOrigin().z() > 0.7)) {
      ROS_INFO_STREAM("Slider Up detected");
      ss << "sliderUp";
      msg.data = ss.str();
      gesturePublisher.publish(msg);
    }
    else if (sliderIsClicked && (transform.getOrigin().z() < -0.7)){
      ROS_INFO_STREAM("Slider down detected");
      ss << "sliderDown";
      msg.data = ss.str();
      gesturePublisher.publish(msg);
    }
    else {
      ROS_INFO_STREAM("No up/down detected. Resetting the click");
      sliderIsClicked = !sliderIsClicked;
    }

}
