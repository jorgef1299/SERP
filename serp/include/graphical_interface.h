#include <gtk/gtk.h>
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// ROS custom services
#include "serp/VelocitySetPoint.h"

#include "boost/date_time/posix_time/posix_time.hpp"

// Global Variables
GtkStyleContext *context;
GtkTextView *log_mensagens;
GtkTextBuffer *log_buffer;
GtkTextIter *log_text_iter;
GtkWidget *button_manual_go;
GtkWidget *button_manual_stop;
GtkRange *range_motor_left;
GtkRange *range_motor_right;
ros::ServiceClient client_velocity_setpoint;
ros::Publisher pub_twist;

enum RobotState {
    Stopped,
    ManualControl
};

struct Robot {
    int8_t motor_left_velocity;
    int8_t motor_right_velocity;
    int8_t motor_left_requested_velocity;
    int8_t motor_right_requested_velocity;
    RobotState state;
} robot;
