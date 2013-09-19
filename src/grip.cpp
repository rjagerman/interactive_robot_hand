#include <ros/ros.h>
#include <ros-assignment1/grip.h>
#include <threemxl/C3mxlROS.h>
#include <iostream>
#include <stdio.h>

void Gripper::init(char *path)
{
  CDxlConfig *config = new CDxlConfig();

  if (path)
  {
    ROS_INFO("Using shared_serial");
    motor_ = new C3mxlROS(path);
  }
  else
  {
    ROS_INFO("Using direct connection");
    motor_ = new C3mxl();

    serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
    serial_port_.set_speed(LxSerial::S921600);
    motor_->setSerialPort(&serial_port_);
  }

  motor_->setConfig(config->setID(109));
  motor_->init(false);
  motor_->set3MxlMode(TORQUE_MODE);

  delete config;
}

void Gripper::spin()
{
  ros::Rate loop_rate(1); //herz;
  bool close = true;

  while (ros::ok())
  {
    std::cin.clear();
    std::cin.get();
    this->grip(close);
    close = !close;
    loop_rate.sleep();
  }

  motor_->setTorque(0);
}

void Gripper::grip(bool close)
{
    motor_->setTorque(close ? -0.005 : 0.005);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_example");

  char *path=NULL;
  if (argc == 2)
    path = argv[1];

  Gripper gripper;

  gripper.init(path);
  gripper.spin();

  return 0;
}
