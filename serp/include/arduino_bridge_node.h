#ifndef SRC_ARDUINO_BRIDGE_NODE_H
#define SRC_ARDUINO_BRIDGE_NODE_H

#include "ros/ros.h"
#include "serp/VelocitySetPoint.h"

// Global Variables
enum mode {
    FixedVelocity,
    Stopped
} operation_mode;

struct Motors {
    int8_t motor_left;
    int8_t motor_right;
} motors;

#endif //SRC_ARDUINO_BRIDGE_NODE_H
