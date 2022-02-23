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
#include "AStar.hpp"



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
};

struct links {
  coordinates point;
  int order;
  bool linked=false;
  int link_end;
};

//X and Y coordinates of 4 corners of block arucos
struct block {
    int id;
    int count;
    int size_aruco;
    coordinates b_sup_left;
    coordinates b_sup_right;
    coordinates b_inf_left;
    coordinates b_inf_right;
    links input1;
    links input2;
    links output1;
    links output2;
    links condition1;
    links condition2;
};

struct combination{
  float number;
  int matrix_pos;
  int dest;
};

//global variables
int size_detect=0;
int pos_list;
int current_ids_size=0;

//paper validation
bool orientation_check = false;
bool pictureValidated = false;
bool vertical = false;
std::vector<int> detections(36, 0); // 36 values equal to 0
int count_stable_frames = 0;
int count_total_arucos = 0;

//combinations of Ks
int num_combinations=0;

//sensor values
int sensor_value_se;
int sensor_value_sd;
int sensor_value_sf;
int sensor_value_st;

//dictionary 4X4
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);


struct orientation_block {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
};



// ---------- LINE DETECTION ---------- (add to separate library)

std::vector<cv::Vec4i> masks;
std::vector<cv::Point2f> crossingPoints;
std::vector<cv::Vec4i> crossingContours;
cv::Mat binaryFinalImage;
