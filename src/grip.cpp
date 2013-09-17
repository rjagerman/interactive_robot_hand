#include <ros/ros.h>
#include <stdio.h>
#include <threemxl/C3mxlROS.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "grip");

    C3mxlROS mxl("grip");
    mxl.init(true);
    printf("3mxl ID: %d\n", mxl.getID());
    printf("3mxl Voltage: %dV\n", mxl.getVoltage());

    return 0;
}
