#include "rpi_cam.h"

void camera_parameters(int type)
{
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++)
    {
      for(int j{0}; j<CHECKERBOARD[0]; j++)
        objp.push_back(cv::Point3f(j,i,0));
    }


    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;

    // Path of the folder containing checkerboard images
    // export SERP_PROJECT_PATH=~/catkin_ws/src/SERP/serp/
    char *project_path = getenv("SERP_PROJECT_PATH");
    char checkerboard_file_name[25] = "include/imgset/*.jpg";
    char checkerboard_file_path[200];
    sprintf(checkerboard_file_path, "%s%s", project_path, checkerboard_file_name);

    cv::glob(checkerboard_file_path, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for(int i{0}; i<images.size(); i++)
    {
      frame = cv::imread(images[i]);
      cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

      // Finding checker board cornersopen
      // If desired number of corners are found in the image then success = true
      success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

      /*
       * If desired number of corner are detected,
       * we refine the pixel coordinates and display them on the images of checker board
      */
      if(success)
      {
        cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

        // refining pixel coordinates for given 2d points.
        cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);

        // Displaying the detected corner points on the checker board
        cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

        objpoints.push_back(objp);
        imgpoints.push_back(corner_pts);
      }

//      cv::imshow("Image",frame);
//      cv::waitKey(1);
    }

    ROS_WARN_STREAM("3D->"<<objpoints.size()<<" 2D->"<<imgpoints.size());

    cv::destroyAllWindows();

    /*
     * Performing camera calibration by passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the detected corners (imgpoints)
    */

    if(type==0) cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cam_info.cameraMatrix, cam_info.distCoeffs, cam_info.R, cam_info.T);
    else if(type==1) cv::fisheye::calibrate(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cam_info.cameraMatrix, cam_info.distCoeffs, cam_info.R, cam_info.T, cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_CHECK_COND |cv::fisheye::CALIB_FIX_SKEW, cv::TermCriteria(cv::TermCriteria::EPS|cv::TermCriteria::MAX_ITER, 30, 1e-6));

//    ROS_WARN_STREAM(cam_info.cameraMatrix);
//    ROS_WARN_STREAM(cam_info.distCoeffs);
//    ROS_WARN_STREAM(cam_info.R);
//    ROS_WARN_STREAM(cam_info.T);
}


//Find the corresponding fisheye output point corresponding to an input cartesian point
cv::Point2f findFisheye(int Xe, int Ye, double R, double Cfx, double Cfy, double He, double We)
{
    cv::Point2f fisheyePoint;
    double theta, r, Xf, Yf; //Polar coordinates

    r = Ye/He*R;
    theta = Xe/We*2.0*PI;
    Xf = Cfx+r*sin(theta);
    Yf = Cfy+r*cos(theta);
    fisheyePoint.x = Xf;
    fisheyePoint.y = Yf;

    return fisheyePoint;
}


void correctImage(int type)
{
    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;

    // Path of the folder containing checkerboard images
    // export SERP_PROJECT_PATH=~/catkin_ws/src/SERP/serp/
    char *project_path = getenv("SERP_PROJECT_PATH");
    char img_file_name[25] = "include/tests/*.jpg";
    char img_file_path[200];
    sprintf(img_file_path, "%s%s", project_path, img_file_name);

    cv::glob(img_file_path, images);


    cv::Mat img = cv::imread(images[3]);
    cv::imshow("Original Image", img);
//    cv::waitKey(0);

    if(type==0)
    {
        // Undistort
        cv::Mat undist_img;

        // Direct Undistort
    //    cv::undistort(img, undist_img, cam_info.cameraMatrix, cam_info.distCoeffs);
    //    cv::fisheye::undistortImage(img, undist_img, cam_info.cameraMatrix, cam_info.distCoeffs);

        // initUndistortRectifyMap With Balance=0.0
        cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);
        cv::Mat map1, map2;
        cv::fisheye::initUndistortRectifyMap(cam_info.cameraMatrix, cam_info.distCoeffs, E, cam_info.cameraMatrix, cv::Size(img.cols, img.rows), CV_16SC2, map1, map2);
        cv::remap(img, undist_img, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::imshow("Corrected Image", undist_img);
        cv::waitKey(0);
    }
    else if(type==1)
    {
        cv::Mat panoramicImage;

        int Hf, Wf, He, We;
        double R, Cfx, Cfy;

        Hf = img.size().height;
        Wf = img.size().width;
        R = Hf/2; //The fisheye image is a square containing a circle so the radius is half of the width or height size
        Cfx = Wf/2; //The fisheye image is a square so the center in x is located at half the distance of the width
        Cfy = Hf/2; //The fisheye image is a square so the center in y is located at half the distance of the height
//        Cfx = cam_info.cameraMatrix.at<double>(0,2);
//        Cfy = cam_info.cameraMatrix.at<double>(1,2);

        He = (int)R;
        We = (int)2*PI*R;

        panoramicImage.create(He, We, img.type());

        for (int Xe = 0; Xe <panoramicImage.size().width; Xe++)
        {
            for (int Ye = 0; Ye <panoramicImage.size().height; Ye++)
            {
                panoramicImage.at<cv::Vec3b>(cv::Point(Xe, Ye)) =    img.at<cv::Vec3b>(findFisheye(Xe, Ye, R, Cfx, Cfy, He, We));
            }
        }

        cv::imshow("Panoramic Image", panoramicImage);
        cv::waitKey(0);
    }
}


int main()
{
    // Type -> 0 (Normal Calibration) , 1 (Fisheye Calibration)
    camera_parameters(1);


//    cv::Mat img = cv::imread("../include/imgset/ck30.jpg", cv::IMREAD_COLOR);
//    cv::imshow("Image 30", img);
//    cv::waitKey(0);

    // Type -> 0 (Undistort Image) , 1 (Transform to Panoramic Image)
    correctImage(1);


    return 0;
}
