#include <ros/ros.h>
#include <interactive_robot_hand/Cylinder.h>
#include <interactive_robot_hand/HandState.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h>

using namespace visualization;

Cylinder::Cylinder() : Node() {
  
  // Create publisher and subscriber
  publisher = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
  eyes_sub = nh.subscribe<interactive_robot_hand::See>("/eyes/see", 1, &Cylinder::see, this);
  hand_sub = nh.subscribe<interactive_robot_hand::HandState>("/hand/state", 1, &Cylinder::grip, this);
  
  // Set cylinder options
  cylinder.type = visualization_msgs::Marker::CYLINDER;
  cylinder.ns = "shapes";
  cylinder.id = 0;
  cylinder.action = visualization_msgs::Marker::ADD;
  cylinder.scale.x = 0.058;
  cylinder.scale.y = 0.058;
  cylinder.scale.z = 0.080;
  cylinder.color.r = 0.350;
  cylinder.color.g = 0.090;
  cylinder.color.b = 0.100;
  cylinder.color.a = 1.0;
  cylinder.pose.position.x = 0.0;
  cylinder.pose.position.y = 0.0;
  cylinder.pose.position.z = 0.0;
  cylinder.lifetime = ros::Duration();
  cylinder.header.stamp = ros::Time::now();
  cylinder.header.frame_id = "/base_link";
  free = true;
  
  // Wait for a subscriber on the /visualization_marker topic to make
  // sure we only create the cylinder once rviz has actually started.
  ros::Rate wait(1);
  while(ros::ok() && publisher.getNumSubscribers() == 0) {
    wait.sleep();
  }
  
  // Create the cylinder
  publisher.publish(cylinder);
  
  // Future updates should only modify the existing cylinder
  cylinder.action = visualization_msgs::Marker::MODIFY;
  
}

Cylinder::~Cylinder() {
  
}

void Cylinder::see(const interactive_robot_hand::See::ConstPtr& message) {
  
  // Make the cylinder fade out if it is out of range
  if(message->distance > 300.0 && message->distance < 400.0) {
    cylinder.color.a = 1.0 - (message->distance-300.0)/100.0;
  } else if(message->distance >= 400.0) {
    cylinder.color.a = 0.0;
  } else {
    cylinder.color.a = 1.0;
  }
  
  // Only update the cylinder to the real world position if it is free and not trapped
  // in the hand, otherwise use a default distance of 55.0 mm
  float position = 0.055;
  if(free) {
     position = message->distance / 1000.0;
  }
  
  // Update the position in rviz
  // The y position gets updated with a smoothing function based on the previous position
  // and new position. This prevents noise from the sensor distance from making the cylinder
  // move irratically.
  cylinder.pose.position.y -= (cylinder.pose.position.y - (position)) / 2;
  cylinder.header.frame_id = "/base_link";
  cylinder.header.stamp = ros::Time::now();
  publisher.publish(cylinder);
  ros::spinOnce();
  
}

void Cylinder::grip(const interactive_robot_hand::HandState::ConstPtr& hand_state) {
  
  // Set the cylinder free, allowing it to move, only if the hand is open
  free = hand_state->open;
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cylinder");
  Cylinder cylinder;
  cylinder.spin();
  return 0;
}
