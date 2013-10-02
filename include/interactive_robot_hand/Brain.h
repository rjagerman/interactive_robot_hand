/**
 * @file
 * @author Rolf Jagerman
 * @version 1.0
 * 
 * @section DESCRIPTION
 * 
 * The brain class represents the logic processing unit. It subscribes to
 * several sensors, obtaining input information. It uses the obtained
 * information to publish events on different topics, in particular it notifies
 * the hand that it should squeeze with a certain force.
 */
#ifndef BRAIN_H
#define BRAIN_H

#include <ros/ros.h>
#include <interactive_robot_hand/Node.h>
#include <interactive_robot_hand/See.h>

namespace robot_hand {
  
  class Brain : public Node {
    
  protected:
    
    ros::Publisher publisher;  ///< The publisher
    ros::Subscriber subscriber;///< The subscriber
    bool active;               ///< Whether the brain is ready to grasp an object
    float force;               ///< The amount of force (in newton) to grasp the object with
    
    /**
     * Sends the correct command to the hand for closing or opening the handler
     * at the specified force.
     * \param force The amount of force (in newton) to apply
     * \param open True to open the hand, false otherwise (default: false)
     */
    void grip(float force, bool open = false);

  public:
    /// Creates a new brain
    Brain();
    
    /// Destroys the brain
    ~Brain();
    
    /// Publishes messages on a timed interval, telling the hand to move
    void spin();
    
    /**
     * Callback handler for seeing objects at a distance. The see message from
     * the eyes describes a distance in millimeters
     * \brief Callback handler for seeing objects at a distance
     * \param message The message send by the eyes
     */
    void see(const interactive_robot_hand::See::ConstPtr& message);

  };

}

#endif // BRAIN_H
