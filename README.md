# rosGestureIdentifier
ROS gesture identifier for system integration project

## Prerequisite
- [Ubuntu Xenial 16.04 LTS](http://releases.ubuntu.com/16.04/)
- [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation)
- Working catkin environment with [catkin workspace](http://wiki.ros.org/ROS/Tutorials)
- [Qt Version 4.8.7](https://doc.qt.io/archives/qt-4.8/index.html) (Other Versions might also work)

## Required ROS Packages
- [Openni launch package to get the kinect running](http://wiki.ros.org/openni_launch)
- [Openni tracker package to get tf data](http://wiki.ros.org/openni_tracker)
- [rosGestureGui package for the qt GUI](https://github.com/yanborn/rosGestureGui/tree/master)

## How to build
Move to your catkin workspace.
Place this package in the src folder of you catkin workspace.

Run the following command in your catkin workspace
> catkin_make

The Ros package will be created. Make sure your workspace is setup properly.

## How to run the package
You have to run the following command
> rosrun rosgestureidentifier rosgestureidentifier_node

## How to use
Run the openni launch and openni tracker packages before running this package.

### Topic names
- openni_depth_frame _-> Topic to receive tf data for left and right hand_
- gestureGui _-> Topic to communicate with the GUI_

### Supported messages
The messages which are send to control the GUI are the following:
- leftHighlighted _-> For highlighting the left drop down menu_
- leftClicked _-> For clicking on the left drop down menu_
- rightHighlighted _-> For highlighting the right drop down menu_
- rightClicked _-> For clicking on the right drop down menu_
- sliderHighlighted _-> For highlighting the slider_
- sliderClicked _-> For clicking on the slider_
- sliderUp _-> For setting the slider one tick up_
- sliderDown _-> For setting the slider one tick down_
- closeGui _-> For closing the GUI_
