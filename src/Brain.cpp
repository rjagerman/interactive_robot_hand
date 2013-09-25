#include <ros/ros.h>
#include <interactive_robot_hand/Brain.h>
#include <std_msgs/Float32.h>
#include <stdio.h>

using namespace interactive_robot_hand;

Brain::Brain() : Node() {
  publisher = nh.advertise<std_msgs::Float32>("grip", 1);
  subscriber = nh.subscribe<std_msgs::Float32>("/eyes/see", 1, &Brain::see, this);
}

Brain::~Brain() {
  
}

void Brain::see(const std_msgs::Float32::ConstPtr& distance) {
  ROS_INFO("Seeing object at %.2fmm", distance->data);
  
  // Grasp the object when it is close
  if(distance->data < 90.0 && active) {
    
    // Create grip force in newtons
    std_msgs::Float32 force;
    force.data = 0.6;
    
    // Publish force
    publisher.publish(force);
    ros::spinOnce();
    ROS_INFO("Published force [%f]", force.data);
    
    active = false;
  }
}

void Brain::spin() {
  
  while(ros::ok()) {
    
    ROS_INFO("Press enter to activate the hand");
    std::cin.clear();
    std::cin.get();
    active = true;
    
    ros::Rate rate(10);
    while(ros::ok() && active) {
      rate.sleep();
      ros::spinOnce();
    }
    
    ROS_INFO("Press enter to let go of the object");
    if(ros::ok()) {
      std::cin.clear();
      std::cin.get();
      std_msgs::Float32 force;
      force.data = -1.0;
      publisher.publish(force);
      ros::spinOnce();
    }
    
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "brain");
  
  Brain* brain = new Brain();
  brain->spin();
  delete brain;
  
  return 0;
}