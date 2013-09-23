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

namespace interactive_robot_hand {
  
  class Brain : public Node {
    
  protected:
    
    ros::Publisher publisher; ///< The publisher

  public:
    /// Creates a new brain
    Brain();
    
    /// Destroys the brain
    ~Brain();
    
    /// Publishes messages on a timed interval, telling the hand to move
    void spin();

  };

}

#endif // BRAIN_H
