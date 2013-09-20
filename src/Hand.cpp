#include <ros-assignment1/Hand.h>

Hand::~Hand(void) {
  this->setCurrent(0);
}

void Hand::grip(float force) {
  ROS_INFO("Gripping the hand with some force");
  this->setCurrent(-1 * force);
}

void Hand::release(void) {
  ROS_INFO("Releasing the hand");
  this->setCurrent(0.2);
}

