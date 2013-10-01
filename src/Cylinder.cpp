#include <ros/ros.h>
#include <interactive_robot_hand/Cylinder.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h>

using namespace visualization;

Cylinder::Cylinder() : Node() {
  
  // Create publisher and subscriber
  publisher = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
  subscriber = nh.subscribe<std_msgs::Float32>("/eyes/see", 1, &Cylinder::see, this);
  
  // Set cylinder options
  cylinder.type = visualization_msgs::Marker::CYLINDER;
  cylinder.ns = "shapes";
  cylinder.id = 0;
  cylinder.action = visualization_msgs::Marker::ADD;
  cylinder.scale.x = 0.058;
  cylinder.scale.y = 0.058;
  cylinder.scale.z = 0.080;
  cylinder.color.r = 0.200;
  cylinder.color.g = 0.500;
  cylinder.color.b = 0.250;
  cylinder.color.a = 1.0;
  cylinder.pose.position.x = 0.0;
  cylinder.pose.position.y = 0.0;
  cylinder.pose.position.z = 0.0;
  cylinder.lifetime = ros::Duration();
  cylinder.header.stamp = ros::Time::now();
  cylinder.header.frame_id = "/base_link";
  
  // Wait for a subscriber on the /visualization_marker topic to make
  // sure we only create the cylinder once rviz has actually started.
  ROS_INFO("Waiting for visualization marker...");
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

void Cylinder::see(const std_msgs::Float32::ConstPtr& distance) {
  
  // Update the cylinder position
  cylinder.pose.position.y = (distance->data/1000.0);
  cylinder.header.frame_id = "/base_link";
  cylinder.header.stamp = ros::Time::now();
  publisher.publish(cylinder);
  ros::spinOnce();
}

void Cylinder::spin() {
  ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cylinder");
  
  Cylinder* cylinder = new Cylinder();
  cylinder->spin();
  delete cylinder;
  
  return 0;
}
