/**
 * @file gestureIdentifier.h
 * @brief Header file for gestureIdentifier class
 *
 * @author Yannick Volkenborn
 * @author Jan Dominik Wisker
 * @author Saidjamol Akbarov
 */

#ifndef GESTUREIDENEITIFER_H
#define GESTUREIDENEITIFER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

/**
 * @brief The gestureIdentifier class
 * @details Class to identify hand gestures from tf with multiple coordinate frames over time
 */
class gestureIdentifier
{
public:
  /**
   * @brief Constructor for gestureIdentifier class
   */
  gestureIdentifier();

  /**
   * @brief Desctuctor for gestureIdentifier class
   */
  ~gestureIdentifier();

  /**
   * @brief startIdentifier Method to start the gesture identifier
   * @details initializes the nodehandle, connects to the gesture gui topic and starts the tf listener loop
   *
   * @return false if ros is not ok anymore
   */
  bool startIdentifier();

private:
  /**
   * @brief Publisher Object to send messages to the gui topic
   */
  ros::Publisher gesturePublisher;
  /**
   * @brief TransformListener Object to receive the tf's from the openni_tracker node
   */
  tf::TransformListener tfListener;

  /**
   * @brief Boolean flag to indicate if the left dropdown menu is highlighted
   */
  static bool leftDropdownIsHighlighted;
  /**
   * @brief Boolean flag to indicate if the left dropdown menu is clicked
   */
  static bool leftDropdownIsClicked;
  /**
   * @brief Boolean flag to indicate if the right dropdown menu is highlighted
   */
  static bool rightDropdownIsHighlighted;
  /**
   * @brief Boolean flag to indicate if the right dropdown menu is clicked
   */
  static bool rightDropdownIsClicked;
  /**
   * @brief Boolean flag to indicate if the slider is highlighted
   */
  static bool sliderIsHighlighted;
  /**
   * @brief Boolean flag to indicate if the slider is clicked
   */
  static bool sliderIsClicked;

  /**
   * @brief Loop to listen to the tfs depending on the boolean flags for the GUI widgets
   * @details The tf loop is running as long as ros::ok() is true. The listening to the tf
   * of the right or left hand depends on the boolean flags indicating if a GUI widget is highlighted/clicked
   *
   * @return false if ros is not ok anymore
   */
  bool runTfLoop();

  /**
   * @brief Checks if the tf received for the left hand indicates that one of the widgets is highlighted
   * @details If the tf received for the left hand indicates that one of the widgets is highlighted, the
   * corresponding message is published via the gestureGui topic to make the highlighting visible in
   * the GUI.
   */
  void checkHighlight();
  /**
   * @brief Checks if the tf received for the right hand indicates that one of the widgets is clicked
   * @details If the tf received for the right hand indicates that one of the widgets is clicked, the
   * corresponding message is published via the gestureGui topic to make the highlighting visible in
   * the GUI.
   */
  void checkClick();
  /**
   * @brief Checks if the tf received for the left hand indicates that there is an up or down movement
   * @details If the tf received for the left hand indicates that there is an up or down movement, the
   * corresponding message is published via the gestureGui topic to make the up or down movement visible
   * in the GUI.
   */
  void checkUpDown();
};

#endif // GESTUREIDENEITIFER_H
