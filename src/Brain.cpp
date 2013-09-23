#include <ros/ros.h>
#include <interactive_robot_hand/Brain.h>
#include <std_msgs/Float32.h>
#include <stdio.h>

using namespace interactive_robot_hand;

Brain::Brain() : Node() {
  publisher = nh.advertise<std_msgs::Float32>("grip", 10);
}

Brain::~Brain() {
  
}

void Brain::spin() {
  
  ros::Rate loop_rate(1);
  float d = 0.3;
  while(ros::ok()) {
    
    // Create grip force in newtons
    d *= -1;
    std_msgs::Float32 force;
    force.data = d;
    
    // Send force to hand
    publisher.publish(force);
    ros::spinOnce();
    ROS_INFO("Published force [%f]", force.data);
    
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "brain");
  
  Brain* brain = new Brain();
  brain->spin();
  
  delete brain;
  return 0;
}