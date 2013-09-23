/**
 * @file
 * @author Rolf Jagerman
 * @version 1.0
 * 
 * @section DESCRIPTION
 * 
 * The hand is a wrapper for the 3mxl controller. It gives easy access to the
 * underlying motor and is able to grip objects with the hand using a specific
 * force. It internally computes the correct current for the given force.
 */
#ifndef HAND_H
#define HAND_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <CDxlGeneric.h>
#include <CDxlGroup.h>
#include <interactive_robot_hand/Node.h>

namespace interactive_robot_hand {

  class Hand : public Node {
    
  protected:
    CDxlGeneric *motor;         ///< The motor interface
    CDxlConfig *config;         ///< The motor configuration
    LxSerial* serial_port;      ///< The serial port
    ros::Subscriber subscriber; ///< The subscriber

  public:
    
    /// Creates a new hand
    Hand();
    
    /// Destroys the hand
    ~Hand();
    
    /// Handles the ros logic
    void spin();
    
    /**
     * Callback function for grip messages specifying a specific force. This
     * function computes the correct current and sends the motor commands to
     * the 3mxl board.
     * \brief Callback function for grip messages
     * \param force The amount of force to apply for the grip
     */
    void grip(const std_msgs::Float32::ConstPtr& force);
    
  };

}

#endif // HAND_H
