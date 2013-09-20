#ifndef ROSMOTOR_H
#define ROSMOTOR_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <CDxlGroup.h>

class RosMotor {

  protected:
    CDxlGeneric *motor;    ///< Motor interface
    CDxlConfig *config;    ///< Motor configuration
    LxSerial* serial_port; ///< Serial port

  public:
    /// Constructor
    RosMotor(LxSerial* serial_port, int id);
    
    /// Destructor
    ~RosMotor();
    
    /// Set current
    void setCurrent(float current);

};

#endif // ROSMOTOR_H
