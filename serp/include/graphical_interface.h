#include <gtk/gtk.h>
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// ROS custom services
#include "serp/VelocitySetPoint.h"

#include "boost/date_time/posix_time/posix_time.hpp"

// Global Variables
GtkTextView *log_mensagens;
GtkTextBuffer *log_buffer;
GtkTextIter *log_text_iter;
ros::ServiceClient client_velocity_setpoint;
ros::Publisher pub_twist;
