#include <ros/ros.h>
#include <ros-assignment1/Hand.h>

int main(int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "assignment1");
  
  // Open serial port
  LxSerial serial_port;
  serial_port.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
  serial_port.set_speed(LxSerial::S921600);

  // Create hand
  Hand* hand = new Hand(&serial_port, 109);
  
  // Grip hand
  hand->grip(0.175);
  sleep(5);
  
  // Release hand
  hand->release();
  sleep(1);
  
  // Cleanup
  delete hand;
  serial_port.port_close();
  return 0;
}
