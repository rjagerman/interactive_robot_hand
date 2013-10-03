#include <ros/ros.h>
#include <interactive_robot_hand/Gripper.h>
#include <interactive_robot_hand/HandState.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h>

using namespace visualization;

Gripper::Gripper() : Node() {
  
  // Create publisher and subscriber
  publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  subscriber = nh.subscribe<interactive_robot_hand::HandState>("/hand/state", 1, &Gripper::grip, this);
  current_base_joint_pos = -0.785;
  current_top_joint_pos = 0.489;
  
}

Gripper::~Gripper() {
  
}

void Gripper::grip(const interactive_robot_hand::HandState::ConstPtr& hand_state) {
  
  // Compute new joint positions based on the hand state
  float wanted_base_joint_pos = -0.110; // max: 0.087
  float wanted_top_joint_pos = 1.431;  // max: 1.431
  if(hand_state->open) {
    wanted_base_joint_pos = -0.785;
    wanted_top_joint_pos = 0.489;
  }
  
  // Animate the joints
  int hertz = 30;
  ros::Rate loop(hertz);
  float start_base_joint_pos = current_base_joint_pos;
  float start_top_joint_pos = current_top_joint_pos;
  for(int i=1; i<=hertz/2; i++) {
    current_base_joint_pos = start_base_joint_pos + i*(wanted_base_joint_pos - start_base_joint_pos)/(hertz/2);
    current_top_joint_pos = start_top_joint_pos + i*(wanted_top_joint_pos - start_top_joint_pos)/(hertz/2);
    update_joints(current_base_joint_pos, current_top_joint_pos);
    loop.sleep();
  }
  
}

void Gripper::update_joints(float base_rotation, float top_rotation) {
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] ="f1_finger_basejoint";
  joint_state.position[0] = base_rotation;
  joint_state.name[1] ="f2_finger_basejoint";
  joint_state.position[1] = base_rotation;
  joint_state.name[2] ="f3_finger_basejoint";
  joint_state.position[2] = base_rotation;
  joint_state.name[3] ="f1_finger_topjoint";
  joint_state.position[3] = top_rotation;
  joint_state.name[4] ="f2_finger_topjoint";
  joint_state.position[4] = top_rotation;
  joint_state.name[5] ="f3_finger_topjoint";
  joint_state.position[5] = top_rotation;
  publisher.publish(joint_state);
  ros::spinOnce();
}

void Gripper::spin() {
  
  // Continuously update the joint positions
  ros::Rate loop(5);
  while(ros::ok()) {
    update_joints(current_base_joint_pos, current_top_joint_pos);
    loop.sleep();
  }
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cylinder");
  
  Gripper gripper;
  gripper.spin();
  
  return 0;
}
