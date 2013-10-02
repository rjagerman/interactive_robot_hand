#include <ros/ros.h>
#include <interactive_robot_hand/Brain.h>
#include <interactive_robot_hand/Grip.h>
#include <interactive_robot_hand/See.h>
#include <std_msgs/Float32.h>
#include <stdio.h>

using namespace robot_hand;

Brain::Brain() : Node() {
  publisher = nh.advertise<interactive_robot_hand::Grip>("grip", 1);
  subscriber = nh.subscribe<interactive_robot_hand::See>("/eyes/see", 1, &Brain::see, this);
  force = 3.0;
}

Brain::~Brain() {
  
}

void Brain::see(const interactive_robot_hand::See::ConstPtr& message) {
  
  // Grab the object when it is close
  if(message->distance < 85.0 && active) {
    
    // Close the hand with the correct amount of force
    grip(force);
    active = false;
    
  }
  
}

void Brain::spin() {
  
  while(ros::ok()) {
    
    // Get user input
    ROS_INFO("Enter the amount of force in newton and press enter to start:");
    std::cin.clear();
    std::cin >> force;
    grip(1.0, true); //< Open the hand
    active = true;
    
    // Wait for hand to close
    ros::Rate rate(10);
    while(ros::ok() && active) {
      rate.sleep();
      ros::spinOnce();
    }
    
    // Hand closed, give the user the option to let go of the object by pressing enter
    ROS_INFO("Press enter to let go of the object");
    if(ros::ok()) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cin.get();
      grip(1.0, true); //< Open the hand
    }
    
  }
}

void Brain::grip(float newton, bool open) {
  
  // Create a grip message
  interactive_robot_hand::Grip grip;
  grip.force = newton;
  grip.open = open;
  
  // Publish it and make sure ROS processes the message
  publisher.publish(grip);
  ros::spinOnce();
  
  // Log the action
  if(open) {
    ROS_INFO("Opening hand with force [%f]", grip.force);
  } else {
    ROS_INFO("Closing hand with force [%f]", grip.force);
  }
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "brain");
  Brain brain;
  brain.spin();
  return 0;
}
