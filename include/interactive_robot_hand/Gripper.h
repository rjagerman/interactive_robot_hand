/**
 * @file
 * @author Rolf Jagerman
 * @version 1.0
 */
#ifndef GRIPPER_H
#define GRIPPER_H

#include <ros/ros.h>
#include <interactive_robot_hand/Node.h>
#include <interactive_robot_hand/Grip.h>
#include <interactive_robot_hand/HandState.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

namespace visualization {
  
  /**
   * The gripper class represents the gripper in the visualization. It reads the status
   * from the hand class and sends joint state messages to update the visualization.
   */
  class Gripper : public Node {
    
  protected:
    
    ros::Publisher publisher;  ///< The publisher for the joints in the visualization
    ros::Subscriber subscriber;///< The subscriber for the hand state messages
    sensor_msgs::JointState joint_state; ///< The message for the state of all the joints
    float current_base_joint_pos; ///< The current base joint position, used for animation
    float current_top_joint_pos;  ///< The current top joint position, used for animation
    
    /**
     * Updates the joints by sending a joint state message to the robot state publisher
     * \param base_rotation The rotation at the base of the finger
     * \param top_rotation The rotation at the top of the finger
     */
    void update_joints(float base_rotation, float top_rotation);

  public:
    /// Creates a new gripper
    Gripper();
    
    /// Destroys the gripper
    ~Gripper();
    
    /**
     * Continuously updates the gripper joints by sending joint state messages
     */
    void spin();
    
    /**
     * Callback handler for the hand's state. This triggers the animation for
     * moving the joints from an open to close state or vice versa.
     * \param hand_state The hand state message send by the hand
     */
    void grip(const interactive_robot_hand::HandState::ConstPtr& hand_state);

  };

}

#endif // GRIPPER_H
