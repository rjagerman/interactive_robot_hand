
#include <ros/ros.h>
#include <interactive_robot_hand/Eyes.h>
#include <interactive_robot_hand/See.h>
#include <interactive_robot_hand/Node.h>
#include <stdexcept>
#include <math.h>

using namespace robot_hand;

Eyes::Eyes() : Node() {
  phidget.init(-1);
  phidget.waitForAttachment(1000);
  if (phidget.getLastError()) {
    ROS_ERROR("Phidget error: %d", phidget.getLastError());
  }
  sensor_id = 0;
  publisher = nh.advertise<interactive_robot_hand::See>("see", 1);
}

Eyes::~Eyes() {
  
}

void Eyes::spin() {
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    
    // Create distance in mm
    interactive_robot_hand::See see;
    see.distance = 26873.548 * pow(phidget.getSensorValue(sensor_id), -0.99925);
    
    // Publish distance
    publisher.publish(see);
    ros::spinOnce();
    ROS_INFO("Published distance [%f]", see.distance);
    
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "eyes");

  // Create eyes
  Eyes* eyes = new Eyes();
  
  // Run
  eyes->spin();
  
  // Delete eyes
  delete eyes;
  
  return 0;
}