#include "rpi_cam.h"

void camera_parameters()
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

//    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cam_info.cameraMatrix, cam_info.distCoeffs, cam_info.R, cam_info.T);
    cv::fisheye::calibrate(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cam_info.cameraMatrix, cam_info.distCoeffs, cam_info.R, cam_info.T, cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC|cv::fisheye::CALIB_FIX_SKEW, cv::TermCriteria(cv::TermCriteria::EPS|cv::TermCriteria::MAX_ITER, 30, 1e-6));

//    ROS_WARN_STREAM(cam_info.cameraMatrix);
//    ROS_WARN_STREAM(cam_info.distCoeffs);
//    ROS_WARN_STREAM(cam_info.R);
//    ROS_WARN_STREAM(cam_info.T);


    // COPY TO correctImage fuction --------------------------------------------------------------------
    cv::Mat img_30 = cv::imread(images[29]);
//    cv::imshow("Image 30", img_30);
//    cv::waitKey(0);

//    cv::imshow("Image 30 - Corners Found", frame);
//    cv::waitKey(0);

    cv::Mat undist_img;
//    cv::undistort(img_30, undist_img, cam_info.cameraMatrix, cam_info.distCoeffs);
//    cv::fisheye::undistortImage(img_30, undist_img, cam_info.cameraMatrix, cam_info.distCoeffs);

    cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(cam_info.cameraMatrix, cam_info.distCoeffs, E, cam_info.cameraMatrix, cv::Size(img_30.cols, img_30.rows), CV_16SC2, map1, map2);
    cv::remap(img_30, undist_img, map1, map2, cv::INTER_LINEAR, CV_HAL_BORDER_CONSTANT);

    cv::imshow("Undistorted Image", undist_img);
    cv::waitKey(0);
}


cv::Mat correctImage(cv::Mat img)
{
}


int main()
{
    camera_parameters();

//    cv::Mat img = cv::imread("../include/imgset/ck30.jpg", cv::IMREAD_COLOR);
//    cv::imshow("Image 30", img);
//    cv::waitKey(0);

//    cv::Mat corrected_img = correctImage(img);
//    cv::imshow("Corrected Image", img);
//    cv::waitKey(0);

    return 0;
}
