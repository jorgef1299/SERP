#include "video_tests.h"


// ---------- CAMERA CALIBRATION + UNDISTORTION ---------- (add to separate library)

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

//    ROS_WARN_STREAM("3D->"<<objpoints.size()<<" 2D->"<<imgpoints.size());

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


cv::Mat correctImage(cv::Mat frame, int type)
{
    // Undistort
    cv::Mat undist_img;

    if(type==0)
    {
        // Direct Undistort
    //    cv::undistort(img, undist_img, cam_info.cameraMatrix, cam_info.distCoeffs);
    //    cv::fisheye::undistortImage(img, undist_img, cam_info.cameraMatrix, cam_info.distCoeffs);

        // initUndistortRectifyMap With Balance=0.0
        cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);
        cv::Mat map1, map2;
        cv::fisheye::initUndistortRectifyMap(cam_info.cameraMatrix, cam_info.distCoeffs, E, cam_info.cameraMatrix, cv::Size(frame.cols, frame.rows), CV_16SC2, map1, map2);
        cv::remap(frame, undist_img, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
//        cv::imshow("Corrected Image", undist_img);
//        cv::waitKey(0);
    }
    else if(type==1)
    {
        // initUndistortRectifyMap With Balance>0.0

        cv::Size dim1 = cv::Size(frame.cols, frame.rows); // Dimension of the original image
        cv::Size dim2 = dim1; // Dimension of the box you want to keep after un-distorting the image
        cv::Size dim3 = dim1; // Dimension of the final box where OpenCV will put the undistorted image

        // When balance = 0,OpenCV will keep the best part of the image for you
        // Whereas balance = 1 tells OpenCV to keep every single pixel of the original image
        double balance = 1.0;

        cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);

        cv::Mat new_K;
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cam_info.cameraMatrix, cam_info.distCoeffs, dim2, E, new_K, balance);

        cv::Mat map1, map2;
        cv::fisheye::initUndistortRectifyMap(cam_info.cameraMatrix, cam_info.distCoeffs, E, new_K, dim3, CV_16SC2, map1, map2);
        cv::remap(frame, undist_img, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }
    else if(type==2)
    {
        cv::Mat panoramicImage;

        int Hf, Wf, He, We;
        double R, Cfx, Cfy;

        Hf = frame.size().height;
        Wf = frame.size().width;
        R = Hf/2; //The fisheye image is a square containing a circle so the radius is half of the width or height size
//        Cfx = Wf/2; //The fisheye image is a square so the center in x is located at half the distance of the width
//        Cfy = Hf/2; //The fisheye image is a square so the center in y is located at half the distance of the height
        Cfx = cam_info.cameraMatrix.at<double>(0,2);
        Cfy = cam_info.cameraMatrix.at<double>(1,2);

        He = (int)R;
        We = (int)2*PI*R;

        panoramicImage.create(He, We, frame.type());

        for (int Xe = 0; Xe <panoramicImage.size().width; Xe++)
        {
            for (int Ye = 0; Ye <panoramicImage.size().height; Ye++)
            {
                panoramicImage.at<cv::Vec3b>(cv::Point(Xe, Ye)) =    frame.at<cv::Vec3b>(findFisheye(Xe, Ye, R, Cfx, Cfy, He, We));
            }
        }

//        cv::imshow("Panoramic Image", panoramicImage);
//        cv::waitKey(0);
    }

    return undist_img;
}



// ---------- ARUCO IDENTIFICATION AND HOMOGRAPHY ---------- (add to separate library)


orientation_block detect_orientation_blocks(std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids)
{
    // Reset flag
    orientation_check=false;

    std::vector<std::vector<cv::Point2f>> orientation_corners;
    orientation_block markers;

    if(ids.size() > 0)
    {
        int corners_detected=0;

        //run through every detected aruco
        for (int i = 0; i < corners.size(); i++)
        {
            if(ids[i] == 28 || ids[i] == 29 || ids[i] == 30 || ids[i] == 31)
            {
                corners_detected++;

                markers.corners.push_back(corners[i]);
                markers.ids.push_back(ids[i]);
            }

            if(corners_detected == 4) orientation_check=true;
        }
    }

    return markers;
}


cv::Point2f extendLines(cv::Mat frame, cv::Point2f point1, cv::Point2f point2, int height, int width)
{
    double x1, y1, x2, y2;
    double m, b;

    x1 = point1.x;
    y1 = point1.y;

    x2 = point2.x;
    y2 = point2.y;

    // Calculate line equation
    m = (y1-y2) / (x1-x2);
    b = y1 - (x1 * m);

//    ROS_WARN_STREAM("y1="<<y1<<" y2="<<y2);
//    ROS_WARN_STREAM("x1="<<x1<<" x2="<<x2);
//    ROS_WARN_STREAM("m="<<m<<" b="<<b);

    // Calculate New Point
    cv::Point2f new_point;

    new_point.x = point1.x - (0.3 * height); // extends point by a ratio
    new_point.y = m * new_point.x + b;
    cv::circle(frame, new_point, 5, cv::Scalar(0,255,255), cv::FILLED, 8, 0);
    cv::line(frame, new_point, point1, (0, 0, 255), 2);

    return new_point;
}


std::vector<cv::Point2f> calculateExtendedPoints(cv::Mat frame, std::vector<cv::Point2f> pts_src, int pos_id, int height, int width)
{
    cv::Point2f id_28 = pts_src[0];
    cv::Point2f id_29 = pts_src[1];
    cv::Point2f id_30 = pts_src[2];
    cv::Point2f id_31 = pts_src[3];

    std::vector<cv::Point2f> new_points;

    if(pos_id == 0) // Paper is vertically oriented and 28 is at the bottom left
    {
        // Draw Extended Lines
        cv::Point2f new_point1 = extendLines(frame, id_28, id_29, height, width);
        cv::Point2f new_point2 = extendLines(frame, id_31, id_30, height, width);
        cv::line(frame, new_point1, new_point2, (0, 0, 255), 2);

        // Substitute exterior detections by extensions
        id_28 = new_point1;
        id_31 = new_point2;
    }
    else if(pos_id == 1) // Paper is vertically oriented and 28 is at the top right
    {
        // Draw Extended Lines
        cv::Point2f new_point1 = extendLines(frame, id_29, id_28, height, width);
        cv::Point2f new_point2 = extendLines(frame, id_30, id_31, height, width);
        cv::line(frame, new_point1, new_point2, (0, 0, 255), 2);

        // Substitute exterior detections by extensions
        id_29 = new_point1;
        id_30 = new_point2;
    }

    // Save new points
    new_points.push_back(id_28);
    new_points.push_back(id_29);
    new_points.push_back(id_30);
    new_points.push_back(id_31);

    return new_points;
}


std::vector<cv::Point2f> calculateNewDimensions(cv::Mat frame, orientation_block markers)
{
    std::vector<cv::Point2f> pts_src;

    cv::Point2f id_28, id_29, id_30, id_31;

//    ROS_WARN_STREAM("Detected orientation blocks:");

    for(int i=0; i<markers.ids.size(); i++)
    {
        // Save corners take keep the most info from the paper (the farthest ones)
        if(markers.ids[i]==28)
        {
            id_28 = cv::Point2f(markers.corners[i][0].x,markers.corners[i][0].y);
            cv::circle(frame, id_28, 5, cv::Scalar(255,255,255), cv::FILLED, 8, 0);
        }
        else if(markers.ids[i]==29)
        {
            id_29 = cv::Point2f(markers.corners[i][3].x,markers.corners[i][3].y);
            cv::circle(frame, id_29, 5, cv::Scalar(255,255,255), cv::FILLED, 8, 0);
        }
        else if(markers.ids[i]==30)
        {
            id_30 = cv::Point2f(markers.corners[i][2].x,markers.corners[i][2].y);
            cv::circle(frame, id_30, 5, cv::Scalar(255,255,255), cv::FILLED, 8, 0);
        }
        else if(markers.ids[i]==31)
        {
            id_31 = cv::Point2f(markers.corners[i][1].x,markers.corners[i][1].y);
            cv::circle(frame, id_31, 5, cv::Scalar(255,255,255), cv::FILLED, 8, 0);
        }
    }

    // Conect real points
    cv::line(frame, id_28, id_29, (255, 0, 0), 2);
    cv::line(frame, id_29, id_30, (255, 0, 0), 2);
    cv::line(frame, id_30, id_31, (255, 0, 0), 2);
    cv::line(frame, id_31, id_28, (255, 0, 0), 2);

    // Points in original frame
    pts_src.push_back(id_28);
    pts_src.push_back(id_29);
    pts_src.push_back(id_30);
    pts_src.push_back(id_31);

    std::vector<cv::Point2f> new_points;

    // Check orientation and calculate new frame dimensions
    int height, width;

    if(abs(id_28.y-id_29.y) < abs(id_28.x-id_29.x))
    {
        vertical = true;
//        ROS_WARN_STREAM("Paper is vertically oriented");

        // Calculate original dimensions
        height = abs(id_29.x-id_28.x);
        width = abs(id_30.y-id_29.y);

        if(id_28.x < id_29.x)
        {
//            ROS_WARN_STREAM("28 is at the bottom left\n");

            // Replace original points by extensions, to account for distortion
            new_points = calculateExtendedPoints(frame, pts_src, 0, height, width);
            id_28 = new_points[0];
            id_29 = new_points[1];
            id_30 = new_points[2];
            id_31 = new_points[3];
        }
        else
        {
//            ROS_WARN_STREAM("28 is at the top right\n");

            // Replace original points by extensions, to account for distortion
            new_points = calculateExtendedPoints(frame, pts_src, 1, height, width);
            id_28 = new_points[0];
            id_29 = new_points[1];
            id_30 = new_points[2];
            id_31 = new_points[3];
        }
    }
    else
    {
        vertical = false;
        ROS_WARN_STREAM("Paper is horizontally oriented");
        ROS_WARN_STREAM("Rotate paper for detection\n");
    }

    return new_points;
}


cv::Mat perspective_correction(cv::Mat original, std::vector<cv::Point2f> new_points)
{
    std::vector<cv::Point2f> pts_dst;

    cv::Mat new_frame;

    int height, width;

    if(vertical)
    {
        // Calculate new dimensions
        height = abs(new_points[1].x-new_points[0].x);
        width = abs(new_points[2].y-new_points[1].y);

        ROS_WARN_STREAM("Height="<<height<<" Width="<<width<<"\n");

        // Points in new frame
        pts_dst.push_back(cv::Point2f(0, 0)); // Matches id_28
        pts_dst.push_back(cv::Point2f(0, height-1)); // Matches id_29
        pts_dst.push_back(cv::Point2f(width-1, height-1)); // Matches id_30
        pts_dst.push_back(cv::Point2f(width-1, 0)); // Matches id_31

        // Calculate Homography
        cv::Mat h = cv::findHomography(new_points, pts_dst);

        // Warp source image to destination based on homography
        cv::warpPerspective(original, new_frame, h, cv::Size(width, height));
    }

    return new_frame;
}


void validatePicture(std::vector<int> ids)
{
    count_frames ++;

    int n_detected = ids.size() - 4; // Ignores the orientation arucos

    if(count_frames < 10)
    {
        if(arucoCount != n_detected)
        {
            count_frames = 1;
            arucoCount = n_detected;
        }
    }
    else if(count_frames == 10)
    {
        ROS_WARN_STREAM("During "<<count_frames<<" frames, detected "<< arucoCount << " arucos\n");

        pictureValidated = true;
        ROS_WARN_STREAM("-- Picture Validated --\n");
    }
    else if(count_frames > 10 || arucoCount != n_detected)
    {
        pictureValidated = false;

        count_frames = 1;
        arucoCount = n_detected;
    }
}



// ---------- BLOCK FORMATION ---------- (add to separate library)


//check what function is associated with the detected aruco
void draw_check_function(int id, int x, int y, cv::InputOutputArray image) {
    if (id == 0) putText(image, "Sum", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 1) putText(image, "*", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 2) putText(image, "-x", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 3) putText(image, "If", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 4) putText(image, "<", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 5) putText(image, ">", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 6) putText(image, "=", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 7) putText(image, "Me", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 8) putText(image, "Md", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 9) putText(image, "Se", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 10) putText(image, "Sd", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 11) putText(image, "Sf", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 12) putText(image, "St", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 13) putText(image, "Temp", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 14) putText(image, "0", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 15) putText(image, "1", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 16) putText(image, "2", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 17) putText(image, "3", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 18) putText(image, "4", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 19) putText(image, "5", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 20) putText(image, "6", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 21) putText(image, "7", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 22) putText(image, "8", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 23) putText(image, "9", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 24) putText(image, ".", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 25) putText(image, "AND", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 26) putText(image, "OR", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 27) putText(image, "mux", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
}


//draw inputs and outputs of blocks (to-do: mux-->condition circle not working)
void draw_points(int id, coordinates sup_esq, coordinates sup_dir, coordinates inf_esq, coordinates inf_dir, cv::InputOutputArray image)
{
    if (id == 0 || id == 1 || id == 3 || id == 4 || id == 5 || id == 6 || id == 13 || id == 25 || id == 26) {
        //2 inputs 1 output
        circle(image, cv::Point(sup_esq.x, sup_esq.y+((inf_esq.y-sup_esq.y)*(0.25))), 9, CV_RGB(0, 0, 255), 1.5); //input top
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75))), 9, CV_RGB(0, 0, 255), 1.5);//input bottom
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
  }
    else if (id == 2 || id == 14 || id == 15 || id == 16 || id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22 || id == 23 || id == 24) {
        //1 input 1 output
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.5))), 9, CV_RGB(0, 0, 255), 1.5); //input top
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
    }
    else if (id == 7 || id == 8) {
        //1 input
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.5))), 9, CV_RGB(0, 0, 255), 1.5); //input top
    }
    else if (id == 9 || id == 10 || id == 11 || id == 12) {
        //1 output
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
    }
    else if (id == 27) {
        //2 inputs 1 output 1 condition
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.25))), 9, CV_RGB(0, 0, 255), 1.5); //input top
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75))), 9, CV_RGB(0, 0, 255), 1.5);//input bottom
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
        circle(image, cv::Point(((inf_dir.x-inf_esq.x)*0.5)+inf_esq.x, inf_dir.y), 9, CV_RGB(0, 0, 255), 1.5); //condition
    }
}


// Draws block body
void draw_full_block(cv::InputOutputArray image_camera, int id, int pos, std::vector<std::vector<cv::Point2f>> corners)
{
        line(image_camera, cv::Point(block_i[pos].b_sup_left.x, block_i[pos].b_sup_left.y), cv::Point(block_i[pos].b_sup_right.x, block_i[pos].b_sup_right.y), cv::Scalar(255), 2, 8, 0); //topo
        line(image_camera, cv::Point(block_i[pos].b_sup_left.x, block_i[pos].b_sup_left.y), cv::Point(block_i[pos].b_inf_left.x, block_i[pos].b_inf_left.y), cv::Scalar(255), 2, 8, 0); //left
        line(image_camera, cv::Point(block_i[pos].b_inf_left.x, block_i[pos].b_inf_left.y), cv::Point(block_i[pos].b_inf_right.x, block_i[pos].b_inf_right.y), cv::Scalar(255), 2, 8, 0); //bottom
        line(image_camera, cv::Point(block_i[pos].b_inf_right.x, block_i[pos].b_inf_right.y), cv::Point(block_i[pos].b_sup_right.x, block_i[pos].b_sup_right.y), cv::Scalar(255), 2, 8, 0); //right

        draw_check_function(id, block_i[pos].b_sup_left.x, block_i[pos].b_sup_left.y - 5, image_camera);

        draw_points(id, block_i[pos].b_sup_left, block_i[pos].b_sup_right, block_i[pos].b_inf_left, block_i[pos].b_inf_right, image_camera);
}


void drawing_functions(cv::InputOutputArray image, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids)
{
    if (ids.size() > 0) {
        current_ids_size = ids.size();
        //aruco::drawDetectedMarkers(image, corners, ids);

        //run through every detected aruco
        for (int i = 0; i < corners.size(); i++)
        {
            draw_full_block(image, ids[i], i, corners);
        }
    }
}


// Count number of times ArUco type was detected
int check_occurences(int id, std::vector<block> vect)
{
  int count=0;

  for (int j = 0; j < vect.size(); j++)
  {
      if(vect[j].id == id) count++;
  }

    return count;
}


//get measures of blocks given the size of aruco that is being read
int get_topmargin(int size_aruco) {
    return size_aruco * (0.2/ 0.8);
}

int get_bottommargin(int size_aruco) {
    return size_aruco * (0.15/ 0.8);
}

int get_topblock(int size_aruco) {
    return size_aruco * (1 / 0.8) * 0.15;
}


// Saves corners of the Block
void corners_blocks(int id, int pos, std::vector<std::vector<cv::Point2f>> corners)
{
  if ((id != 28) && (id != 29) && (id != 30) && (id<31))
  {
      block_i.push_back(block());

      block_i[pos].count=check_occurences(id,block_i)+1;

      block_i[pos].id=id;



      block_i[pos].size_aruco = corners[pos][1].x - corners[pos][0].x;
      block_i[pos].b_sup_left.x = corners[pos][0].x - get_topblock(block_i[pos].size_aruco);
      block_i[pos].b_sup_left.y = corners[pos][0].y - get_topmargin(block_i[pos].size_aruco);

      block_i[pos].b_sup_right.x = corners[pos][1].x + get_topblock(block_i[pos].size_aruco);
      block_i[pos].b_sup_right.y = corners[pos][1].y - get_topmargin(block_i[pos].size_aruco);

      block_i[pos].b_inf_left.x = corners[pos][3].x - get_topblock(block_i[pos].size_aruco);
      block_i[pos].b_inf_left.y = corners[pos][3].y + get_bottommargin(block_i[pos].size_aruco);

      block_i[pos].b_inf_right.x = corners[pos][2].x + get_topblock(block_i[pos].size_aruco);
      block_i[pos].b_inf_right.y = corners[pos][2].y + get_bottommargin(block_i[pos].size_aruco);
  }

}


// Saves Block Inputs and Outputs
void save_in_out(cv::InputOutputArray image, coordinates sup_esq, coordinates sup_dir, coordinates inf_esq, coordinates inf_dir, int pos_list, int id)
{

    masks.push_back(cv::Vec4i());
    //min_x max_x min_y max_y
    masks[pos_list][0]=sup_esq.x;
    masks[pos_list][1]=inf_dir.x;
    masks[pos_list][2]=sup_esq.y;
    masks[pos_list][3]=inf_dir.y;

    if (id == 0 || id == 1 || id == 3 || id == 4 || id == 5 || id == 6 || id == 13 || id == 25 || id == 26)
    {
        //2 inputs 1 output

        block_i[pos_list].id = id;
        block_i[pos_list].outputs.x = sup_dir.x;
        block_i[pos_list].outputs.y = sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5);
        block_i[pos_list].input1.x = sup_esq.x;
        block_i[pos_list].input1.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.25));
        block_i[pos_list].input2.x = sup_esq.x;
        block_i[pos_list].input2.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75));
    }
    else if (id == 2 || id == 14 || id == 15 || id == 16 || id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22 || id == 23 || id == 24)
    {
        //1 input 1 output

        block_i[pos_list].id = id;
        block_i[pos_list].outputs.x = sup_dir.x;
        block_i[pos_list].outputs.y = sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5);
        block_i[pos_list].input1.x = sup_esq.x;
        block_i[pos_list].input1.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.5));
    }
    else if (id == 7 || id == 8)
    {
        //1 input

        block_i[pos_list].id = id;
        block_i[pos_list].input1.x = sup_esq.x;
        block_i[pos_list].input1.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.5));
    }
    else if (id == 9 || id == 10 || id == 11 || id == 12)
    {
        //1 output

        block_i[pos_list].id = id;
        block_i[pos_list].outputs.x = sup_dir.x;
        block_i[pos_list].outputs.y = sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5);
    }
    else if (id == 27)
    {
        //2 inputs 1 output 1 condition

        block_i[pos_list].id = id;
        block_i[pos_list].outputs.x = sup_dir.x;
        block_i[pos_list].outputs.y = sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5);
        block_i[pos_list].input1.x = sup_esq.x;
        block_i[pos_list].input1.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.25));
        block_i[pos_list].input2.x = sup_esq.x;
        block_i[pos_list].input2.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75));
        block_i[pos_list].condition.x = ((inf_dir.x-inf_esq.x)*0.5)+inf_esq.x;
        block_i[pos_list].condition.y = inf_dir.y;
    }

}


// Saves every Block's Corners and I/Os
void saving_coordinates(cv::InputOutputArray image, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids)
{
    if (ids.size() > 0)
    {
        current_ids_size = ids.size();

        //run through every detected aruco
        for (int i = 0; i < corners.size(); i++)
        {
            corners_blocks(ids[i], i, corners);

            save_in_out(image,block_i[i].b_sup_left, block_i[i].b_sup_right, block_i[i].b_inf_left, block_i[i].b_inf_right, i, ids[i]);
        }
    }
}


std::string check_function(int id)
{
    if (id == 0) return "Aruco SUM";
    else if (id == 1)return "Aruco PRODUCT";
    else if (id == 2) return "Aruco INVERSE";
    else if (id == 3) return "Aruco IF";
    else if (id == 4) return "Aruco LESS THAN";
    else if (id == 5) return "Aruco MORE THAN";
    else if (id == 6) return "Aruco EQUAL";
    else if (id == 7) return "Aruco LEFT MOTOR";
    else if (id == 8) return "Aruco RIGHT MOTOR";
    else if (id == 9) return "Aruco LEFT SENSOR";
    else if (id == 10) return "Aruco RIGHT SENSOR";
    else if (id == 11) return "Aruco FRONT SENSOR";
    else if (id == 12) return "Aruco REAR SENSOR";
    else if (id == 13) return "Aruco TIMER";
    else if (id == 14) return "Aruco NUMBER 0";
    else if (id == 15) return "Aruco NUMBER 1";
    else if (id == 16) return "Aruco NUMBER 2";
    else if (id == 17) return "Aruco NUMBER 3";
    else if (id == 18) return "Aruco NUMBER 4";
    else if (id == 19) return "Aruco NUMBER 5";
    else if (id == 20) return "Aruco NUMBER 6";
    else if (id == 21) return "Aruco NUMBER 7";
    else if (id == 22) return "Aruco NUMBER 8";
    else if (id == 23) return "Aruco NUMBER 9";
    else if (id == 24) return "Aruco DOT";
    else if (id == 25) return "Aruco AND";
    else if (id == 26) return "Aruco OR";
    else if (id == 27) return "Aruco MUX";
}


std::vector<block> put_arucos_order(std::vector<block> blocks )
{
  std::vector<block> new_order_block;

  //copy.assign(blocks.begin(),blocks.end());

  int min;
  int index_min;


  while(blocks.size()>0)
  {
    //cout  << "Iteration: " << blocks.size() << "\n";

    min=blocks[0].b_sup_left.x;
    index_min=0;
    for(int i=0; i<blocks.size(); i++)
    {
      //cout << blocks[i].id << "  " << check_function(blocks[i].id) << "\n";

      if(blocks[i].b_sup_left.x < min)
      {
        min = blocks[i].b_sup_left.x;
        index_min=i;
      }
    }
    //cout << min << "  IMHERE\n";

    new_order_block.push_back(blocks[index_min]);

    std::vector<block>::iterator it;
    it= blocks.begin()+index_min;
    blocks.erase(it);
  }

  return new_order_block;
}


void DebugBlocks()
{
    for (int j = 0; j < block_in_order.size(); j++)
    {
        ROS_WARN_STREAM("ARUCO OF ID " <<  block_in_order[j].id << "(" << block_in_order[j].count << ") --> " << check_function(block_in_order[j].id) << " : Outputs-->" << block_in_order[j].outputs.x << "  " << block_in_order[j].outputs.y << "  " << "\n");
        ROS_WARN_STREAM("                                    "  << " : Input1-->" << block_in_order[j].input1.x << "  " << block_in_order[j].input1.y << "  " << "\n");
        ROS_WARN_STREAM("                                    "  << " : Input2-->" << block_in_order[j].input2.x << "  " << block_in_order[j].input2.y << "  " << "\n");
        ROS_WARN_STREAM("MASK    minx " << masks[j][0] << "    maxx " << masks[j][1] << "    miny " << masks[j][2] << "    maxy " << masks[j][3] << "\n");
        ROS_WARN_STREAM("-----------------------------------------------------------------------------------------\n");
    }
}



// ---------- LINE DETECTION ---------- (add to separate library)




// ---------- LINE DETECTION LOGIC ----------
void detectAndInterpret_Lines(cv::Mat new_frame, cv::Ptr<cv::aruco::Dictionary> dict)
{
    cv::Mat paper = new_frame.clone();
    cv::Mat paperDrawn = new_frame.clone();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(paper, dict, corners, ids);


    // Block Formation

    saving_coordinates(paper,corners,ids);

    drawing_functions(paperDrawn,corners,ids);

    block_in_order = put_arucos_order(block_i);

//    DebugBlocks();


    // Line Detection

    graph g(block_in_order.size());

    //Add links to graph
//    links(paper,g,block_in_order);

    cv::imshow("Paper", paperDrawn);
    cv::waitKey(1);
}



// ---------- PAPER LOGIC (MAIN FUNCTION) ----------
void detectAndInterpret_Paper(cv::Mat frame, cv::Ptr<cv::aruco::Dictionary> dict)
{
    cv::Mat frameCopy;

    frame.copyTo(frameCopy);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(frame, dict, corners, ids); // corners returned clockwise, starting with top lef

    cv::aruco::drawDetectedMarkers(frameCopy, corners, ids);

    orientation_block markers = detect_orientation_blocks(corners, ids);

    cv::Mat original = frame.clone();

    if(markers.ids.size()==4 && orientation_check)
    {
        validatePicture(ids);

        std::vector<cv::Point2f> new_points = calculateNewDimensions(frameCopy, markers);

        if(pictureValidated)
        {
            cv::Mat new_frame = perspective_correction(original, new_points);

            detectAndInterpret_Lines(new_frame, dict);
        }
    }
    else
    {
        // Paper not detected and picture validation count is interrupted
        count_frames = 0;
        arucoCount = 0;
    }

    imshow("out", frameCopy);
    cv::waitKey(1);
}



// ---------- MAIN FOR TESTS ----------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_tests_node");
    ros::NodeHandle n_public;


//    // Get Camera Calibration Parameters
//    camera_parameters(1);

    // Create a VideoCapture object and open the input file
      // If the input is the web camera, pass 0 instead of the video file name
      cv::VideoCapture cap("../catkin_ws/src/SERP/serp/include/tests/test2.mp4");

      // Check if camera opened successfully
      if(!cap.isOpened())
      {
        ROS_WARN_STREAM("Error opening video stream or file");
        return -1;
      }

    while(ros::ok())
    {
        cv::Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty()) break;

        // Display the resulting frame
//        cv::imshow("Original Frame", frame);
//        cv::waitKey(1);

//        // Correct Frame Distortion
//        cv::Mat undist_frame = correctImage(frame, 1);
//        cv::imshow("Fixed Frame", undist_frame);
//        cv::waitKey(1);

        // ArUco Identification
        detectAndInterpret_Paper(frame, dictionary);

        ros::spinOnce();
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}
