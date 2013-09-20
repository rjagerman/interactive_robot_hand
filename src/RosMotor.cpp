#include <ros-assignment1/RosMotor.h>
#include <threemxl/C3mxlROS.h>

RosMotor::RosMotor (LxSerial* serial_port, int id) {
  
  // Create configuration
  config = new CDxlConfig();
  
  // Create 3mxl motor
  motor = new C3mxl();
  
  // Set parameters for motor
  motor->setSerialPort(serial_port);
  motor->setConfig(config->setID(id));
  motor->init(false);
  motor->set3MxlMode(CURRENT_MODE);
  
  // Notify user
  ROS_INFO("Created ROS Motor on serial port");
  delete config;
}

RosMotor::~RosMotor() {
  delete motor;
}

void RosMotor::setCurrent(float current) {
  motor->setCurrent(current);
}
