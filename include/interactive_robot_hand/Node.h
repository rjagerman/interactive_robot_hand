#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>

namespace interactive_robot_hand {

  class Node {
    
  protected:
    ros::NodeHandle nh;
    
  public:
    
    Node() : nh("~") {
      
    }
    
    virtual ~Node() {
      nh.shutdown();
    }
    
    virtual void spin() = 0;
    
  };

}

#endif // EYES_H
