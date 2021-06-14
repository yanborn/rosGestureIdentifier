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
  gesturePublisher = nh.advertise<std_msgs::String>("gestureGui", 1);

  return runTfLoop();
}

bool
gestureIdentifier::runTfLoop()
{
  ros::Rate loopRate(1.0);
  std_msgs::String msg;
  std::stringstream ss;

  while(ros::ok())
  {
    tf::StampedTransform transform;
    try {
      tfListener.lookupTransform("/openni_depth_frame", "/left_hand_1", ros::Time(0), transform);

      if(transform.getOrigin().y() <= -0.2)
      {
        ss << "leftHighlighted";
        msg.data = ss.str();
        gesturePublisher.publish(msg);
      }
      else if(transform.getOrigin().y() > -0.1 && transform.getOrigin().y() < 0.1)
      {
        ss << "rightHighlighted";
        msg.data = ss.str();
        gesturePublisher.publish(msg);
        //gesturePublisher.publish("rightHighlighted");
      }
      else if(transform.getOrigin().y() > 0.2)
      {
        ss << "sliderHighlighted";
        msg.data = ss.str();
        gesturePublisher.publish(msg);
        //gesturePublisher.publish("sliderHighlighted");
      }
    } catch (tf::TransformException &e) {
      ROS_ERROR_STREAM("Error transforming tf.");
    }

    ros::spinOnce();

    loopRate.sleep();
  }

  return 0;
}
