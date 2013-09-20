#ifndef HAND_H
#define HAND_H

#include <ros-assignment1/RosMotor.h>

class Hand : public RosMotor {

public:
  Hand(LxSerial* serial_port, int id) : RosMotor(serial_port, id) {}
  ~Hand();
  
  void grip(float force);
  void release();
  
};

#endif // HAND_H
