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
    CDxlGeneric *motor;    ///< Motor interface
    CDxlConfig *config;    ///< Motor configuration
    LxSerial* serial_port; ///< Serial port
    
    ros::Subscriber subscriber;

  public:
    Hand();
    ~Hand();
    
    void spin();
    void grip(const std_msgs::Float32::ConstPtr& force);
    
  };

}

#endif // HAND_H
