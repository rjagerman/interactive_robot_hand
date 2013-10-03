/**
 * @file
 * @author Rolf Jagerman
 * @version 1.0
 */
#ifndef CYLINDER_H
#define CYLINDER_H

#include <ros/ros.h>
#include <interactive_robot_hand/Node.h>
#include <visualization_msgs/Marker.h>
#include <interactive_robot_hand/See.h>
#include <interactive_robot_hand/HandState.h>

namespace visualization {
  
  /**
   * The cylinder class represents a cylinder in the visualization. It reads distance
   * from the eyes and places a cylinder at the correct location in the visualization.
   */
  class Cylinder : public Node {
    
  protected:
    
    ros::Publisher publisher;            ///< The publisher for the visualization marker messages
    ros::Subscriber eyes_sub;            ///< The subscriber for the eyes' see messages
    visualization_msgs::Marker cylinder; ///< The cylinder for the visualization
    ros::Subscriber hand_sub;            ///< The subscriber for the handstate messages
    bool free;                           ///< Describes whether the cylinder is free or trapped in the hand

  public:
    /// Creates a new cylinder
    Cylinder();
    
    /// Destroys the cylinder
    ~Cylinder();
    
    /**
     * Callback handler for seeing objects at a distance. The see message from
     * the eyes describes a distance in millimeters
     * \brief Callback handler for seeing objects at a distance
     * \param message The message send by the eyes
     */
    void see(const interactive_robot_hand::See::ConstPtr& message);
    
    /**
     * Callback handler for the current hand state when it grips. This places
     * the cylinder in a fixed location if the hand is closed, and frees it
     * when the hand is open.
     * \param hand_state The hand state message send by the hand
     */
    void grip(const interactive_robot_hand::HandState::ConstPtr& hand_state);

  };

}

#endif // CYLINDER_H
