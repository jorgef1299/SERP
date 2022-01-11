#include <gtk/gtk.h>
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"

// ROS custom services
#include "serp/VelocitySetPoint.h"

#include "boost/date_time/posix_time/posix_time.hpp"
#include <unistd.h>

// Global Variables
GtkStyleContext *context;
GtkTextView *log_mensagens;
GtkTextBuffer *log_buffer;
GtkTextIter *log_text_iter;
GtkWidget *button_manual_go;
GtkWidget *button_manual_stop;
GtkRange *range_motor_left;
GtkRange *range_motor_right;
GtkLabel  *label_battery;
GtkImage *camera_image_frame;

ros::ServiceClient client_velocity_setpoint;
ros::ServiceClient client_battery_level;
ros::Publisher pub_twist;

image_transport::ImageTransport *it;

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
    int8_t battery_level;
} robot;

bool display_camera;
