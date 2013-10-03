/**
 * @file
 * @author Rolf Jagerman
 * @version 1.0
 */
#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>

/**
 * The node is an abstract base class for ROS nodes. It sets up a node handle
 * and offers  default spin() method, which can be overridden. Overriding the
 * constructor makes it possible to customize the initialization of the class.
 */
class Node {
    
  protected:
    
    /// The node handle
    ros::NodeHandle nh;
    
  public:
    
    /// Creates a new node
    Node() : nh("~") {
      
    }
    
    /// Destroys an existing node
    virtual ~Node() {
      nh.shutdown();
    }
    
    /**
     * This method should contain the ROS logic.
     * The default implementation places ROS in a message handling
     * loop (ros::spin()).
     */
    virtual void spin() {
      ros::spin();
    }
    
};

#endif // NODE_H
