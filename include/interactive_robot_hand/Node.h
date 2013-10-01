/**
 * @file
 * @author Rolf Jagerman
 * @version 1.0
 * 
 * @section DESCRIPTION
 * 
 * The node is an abstract base class for ROS nodes. It sets up a node handle
 * and offers an empty spin() method, which can be overridden. Overriding the
 * constructor makes it possible to customize the initialization of the class.
 */
#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>

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
    
    /// A method that should contain the ROS logic
    virtual void spin() = 0;
    
};

#endif // NODE_H
