#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>


// GLOBAL VARIABLES

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{5,8}; // Number of corners (vertical, horizontal)

typedef struct {
    cv::Mat cameraMatrix, distCoeffs, R, T;
} camera_info;

camera_info cam_info;

const double PI = 3.141592653589793;
