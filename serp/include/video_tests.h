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
    coordinates input1;
    coordinates input2;
    coordinates outputs;
    coordinates condition;
};

//global variables
int size_detect=0;
int pos_list;
int current_ids_size=0;
bool orientation_check = false;
int count_frames = 0;
int arucoCount = 0;
bool pictureValidated = false;
bool vertical = false;

//dictionary 4X4
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);


struct orientation_block {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
};



// ---------- LINE DETECTION ---------- (add to separate library)

std::vector<cv::Vec4i> masks;

class graph
{
    public:

        std::list<int> *adjlist;
        int n;

        graph(int v)
        {
            adjlist=new std::list<int> [v];
            n=v;
        }

        void addedge(int u,int v,bool bi)
        {
            adjlist[u].push_back(v);

            if(bi) adjlist[v].push_back(u);
        }

//        void print()
//        {
//            for(int i=0;i<n;i++)
//            {
//                std::cout<< "ARUCO ID: "<<block_in_order[i].id<<"  "<<i<<" -->";
//                for(auto it:adjlist[i]){
//                    std::cout<<it<<" ";
//                }
//                std::cout<<std::endl;
//            }
//            std::cout<<std::endl;
//        }
};
