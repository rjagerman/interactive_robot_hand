/**
 * @file
 * @author Rolf Jagerman
 * @version 1.0
 * 
 * @section DESCRIPTION
 * 
 * The cylinder class represents a cylinder in the visualization. It reads distance
 * from the eyes and places a cylinder at the correct location in the visualization.
 */
#ifndef CYLINDER_H
#define CYLINDER_H

#include <ros/ros.h>
#include <interactive_robot_hand/Node.h>
#include <visualization_msgs/Marker.h>
#include <interactive_robot_hand/See.h>
#include <interactive_robot_hand/HandState.h>

namespace visualization {
  
  class Cylinder : public Node {
    
  protected:
    
    ros::Publisher publisher;  ///< The publisher
    ros::Subscriber subscriber;///< The subscriber
    visualization_msgs::Marker cylinder; ///< The cylinder
    ros::Subscriber hand_sub;  ///< The hand subscriber
    bool free;                 ///< Describes whether the cylinder is free or trapped in the hand

  public:
    /// Creates a new cylinder
    Cylinder();
    
    /// Destroys the cylinder
    ~Cylinder();
    
    /// Publishes messages on a timed interval, telling the hand to move
    void spin();
    
    /**
     * Callback handler for seeing objects at a distance. The see message from
     * the eyes describes a distance in millimeters
     * \brief Callback handler for seeing objects at a distance
     * \param message The message send by the eyes
     */
    void see(const interactive_robot_hand::See::ConstPtr& message);
    
    /**
     * Callback handler for the current hand state when it grips
     * \param hand The hand state
     */
    void grip(const interactive_robot_hand::HandState::ConstPtr& hand);

  };

}

#endif // CYLINDER_H
