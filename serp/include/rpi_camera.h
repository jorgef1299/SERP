#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

// Global variables
enum state {
    NormalOperation,
    ReadProgrammingSheet,
    DetectObstacles
} camera_state;