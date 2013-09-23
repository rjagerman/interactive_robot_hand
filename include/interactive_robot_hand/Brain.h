#ifndef BRAIN_H
#define BRAIN_H

#include <ros/ros.h>
#include <interactive_robot_hand/Node.h>

namespace interactive_robot_hand {

  class Brain : public Node {
    
  protected:
    ros::Publisher publisher;

  public:
    Brain();
    ~Brain();
    
    void spin();

  };

}

#endif // BRAIN_H
