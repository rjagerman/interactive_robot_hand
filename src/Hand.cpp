#include <ros/ros.h>
#include <interactive_robot_hand/Hand.h>
#include <interactive_robot_hand/HandState.h>
#include <interactive_robot_hand/Grip.h>
#include <threemxl/C3mxlROS.h>

using namespace robot_hand;

Hand::Hand() : Node() {
  
  // Create configuration
  config = new CDxlConfig();
  
  // Open serial port
  serial_port = new LxSerial();
  serial_port->port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
  serial_port->set_speed(LxSerial::S921600);
  
  // Create 3mxl motor
  motor = new C3mxl();
  
  // Set parameters for motor
  motor->setSerialPort(serial_port);
  motor->setConfig(config->setID(109));
  motor->init(false);
  motor->set3MxlMode(CURRENT_MODE);
  
  // Start subscription
  subscriber = nh.subscribe<interactive_robot_hand::Grip>("/brain/grip", 1, &Hand::grip, this);
  publisher = nh.advertise<interactive_robot_hand::HandState>("state", 1);

  // Notify user
  ROS_INFO("Created hand");
}

Hand::~Hand(void) {
  motor->setCurrent(0);
  serial_port->port_close();
  delete config;
  delete motor;
  delete serial_port;
  ROS_INFO("Deleted hand");
}

void Hand::spin(void) {
  ros::spin();
}

void Hand::grip(const interactive_robot_hand::Grip::ConstPtr& msg) {
  
  // Get the message force, invert it when opening the hand
  float force = msg->force;
  if(msg->open) {
    force *= -1;
  }
  
  // Calculate the correct current based on the force and send it to the motor
  float current = -(0.030246 * force - 0.0270571);
  motor->setCurrent(current);
  
  // Publish the hand state message
  state.open = msg->open;
  publisher.publish(state);
  ros::spinOnce();
  
  // Notify user
  ROS_INFO("Generating current [%f]", current);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hand");
  Hand hand;
  hand.spin();
  return 0;
}

