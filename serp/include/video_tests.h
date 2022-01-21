#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <numeric>



// GLOBAL VARIABLES


// ---------- CAMERA CALIBRATION + UNDISTORTION ---------- (add to separate library)

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{5,8}; // Number of corners (vertical, horizontal)

typedef struct {
    cv::Mat cameraMatrix, distCoeffs, R, T;
} camera_info;

camera_info cam_info;

const double PI = 3.141592653589793;



// ---------- ARUCO IDENTIFICATION ---------- (add to separate library)

//X and Y coordinates of left corners of orientation arucos
struct coordinates {
    int x;
    int y;
} o_sup_left, o_inf_left, o_inf_right;

//X and Y coordinates of 4 corners of block arucos
struct block {
    coordinates b_sup_left;
    coordinates b_sup_right;
    coordinates b_inf_left;
    coordinates b_inf_right;
} block_i;

//global variables
int current_ids_size=0;
int size_aruco;
bool orientation_check = false;
int count_frames = 0;
int arucoCount = 0;
bool pictureValidated = false;

//dictionary 4X4
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);


struct orientation_block {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
};
