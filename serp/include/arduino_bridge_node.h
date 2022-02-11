#ifndef SRC_ARDUINO_BRIDGE_NODE_H
#define SRC_ARDUINO_BRIDGE_NODE_H

#include "ros/ros.h"
#include "serp/VelocitySetPoint.h"
#include "std_srvs/Trigger.h"
#include <std_msgs/String.h>

// Global Variables
enum RobotState {
    Stopped,
    ManualControl,
    Executing
} robot_state;

struct Robot {
    int8_t motor_left_velocity;
    int8_t motor_right_velocity;
    int8_t battery_level;
} robot;

#endif //SRC_ARDUINO_BRIDGE_NODE_H
