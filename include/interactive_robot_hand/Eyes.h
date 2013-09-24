/**
 * @file
 * @author Rolf Jagerman
 * @version 1.0
 * 
 * @section DESCRIPTION
 * 
 * The eyes are a wrapper for the phidget interface kit. It gives easy access
 * to the underlying sensor and contains a formula to convert the sensor value
 * to mm's. The distance gets published on the channel see.
 */
#ifndef EYES_H
#define EYES_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <interactive_robot_hand/Node.h>
#include <phidget_ik/phidget_ik.h>

namespace interactive_robot_hand {

  class Eyes : public Node {
    
  protected:
    PhidgetIK phidget;         ///< The phidget interface
    int sensor_id;             ///< The sensor id
    ros::Publisher publisher;  ///< The publisher

  public:
    
    /// Creates new eyes
    Eyes();
    
    /// Destroys the eyes
    ~Eyes();
    
    /// Handles the ros logic
    void spin();
    
  };

}

#endif // EYES_H
