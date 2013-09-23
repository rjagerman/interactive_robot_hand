#include <ros/ros.h>
#include <interactive_robot_hand/Hand.h>
#include <threemxl/C3mxlROS.h>
#include <std_msgs/Float32.h>

using namespace interactive_robot_hand;

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
  subscriber = nh.subscribe("/brain/grip", 10, &Hand::grip, this);
  
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

void Hand::grip(const std_msgs::Float32::ConstPtr& msg) {
  ROS_INFO("Received force [%f]", msg->data);
  motor->setCurrent(msg->data);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hand");

  // Create hand
  Hand* hand = new Hand();
  
  // Run
  hand->spin();
  
  // Delete hand
  delete hand;
  
  return 0;
}

