/**
 * @file
 * @author Rolf Jagerman
 * @version 1.0
 */
#ifndef EYES_H
#define EYES_H

#include <ros/ros.h>
#include <interactive_robot_hand/Node.h>
#include <interactive_robot_hand/See.h>
#include <phidget_ik/phidget_ik.h>

namespace robot_hand {

  /**
   * The eyes are a wrapper for the phidget interface kit. It gives easy access
   * to the underlying sensor and contains a formula to convert the sensor value
   * to mm's. The distance gets published on the channel see.
   */
  class Eyes : public Node {
    
  protected:
    PhidgetIK phidget;         ///< The phidget interface
    int sensor_id;             ///< The sensor id
    ros::Publisher publisher;  ///< The publisher for the see messages

  public:
    
    /// Creates new eyes
    Eyes();
    
    /// Destroys the eyes
    ~Eyes();
    
    /**
     * Continuously reads information from the phidget_ik board and sends messages
     * over the /eyes/see topic stating there is an object at a specific distance
     */
    void spin();
    
  };

}

#endif // EYES_H
