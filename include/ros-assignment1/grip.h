#ifndef __ROSASSIGNMENT1_GRIP_H
#define __ROSASSIGNMENT1_GRIP_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <CDxlGroup.h>

/// Usage example for CDynamixel and CDynamixelROS
class Gripper
{
  protected:
    ros::NodeHandle nh_;   ///< ROS node handle
    CDxlGeneric *motor_;   ///< Motor interface
    CDxlGroup *motors_;    ///< Multiple motor interface
    LxSerial serial_port_; ///< Serial port interface

  public:
    /// Constructor
    Gripper() : nh_("~"), motor_(NULL) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~Gripper()
    {
      if (motor_)
        delete motor_;
      if (serial_port_.is_port_open())
        serial_port_.port_close();
          
      nh_.shutdown();
    }

    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    
    /// Spin
    /** Alternatively drives the motor clockwise and counterclockwise */
    void spin();

    /// Activate the gripper hand
    /** \param closed Whether to close (true) the hand or open (false) the hand */
    void grip(bool closed);
};    

#endif /* __ROSASSIGNMENT1_GRIP_H */

