#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/SetBool.h>
#include "serp/Sensors.h"
#include <raspicam/raspicam_cv.h>

// Global variables
bool must_publish_camera_data;

