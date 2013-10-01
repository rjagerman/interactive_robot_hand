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
#include <std_msgs/Float32.h>

namespace visualization {
  
  class Cylinder : public Node {
    
  protected:
    
    ros::Publisher publisher;  ///< The publisher
    ros::Subscriber subscriber;///< The subscriber
    visualization_msgs::Marker cylinder; ///< The cylinder

  public:
    /// Creates a new cylinder
    Cylinder();
    
    /// Destroys the cylinder
    ~Cylinder();
    
    /// Publishes messages on a timed interval, telling the hand to move
    void spin();
    
    /**
     * Callback handler for seeing objects at a distance. The distance is in mm's and
     * is send by the eyes.
     * \brief Callback handler for seeing objects at a distance
     * \param distance The distance at which the object is perceived (in mm's)
     */
    void see(const std_msgs::Float32::ConstPtr& distance);

  };

}

#endif // CYLINDER_H
