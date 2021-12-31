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



// ---------- ARUCO IDENTIFICATION ---------- (add to separate library)

//save coordinates of orientation arucos
void orientation(int id ,int x_corner, int y_corner)
{
    if (id == 28)
    {
        o_sup_left.x = x_corner;
        o_sup_left.y = y_corner;
    }

    else if (id == 29)
    {
        o_inf_left.x = x_corner;
        o_inf_left.y = y_corner;
    }

    else if (id == 30)
    {
        o_inf_right.x = x_corner;
        o_inf_right.y = y_corner;
    }
    else return;
}


//threshold for checking orientation
bool inRange(unsigned low, unsigned high, unsigned x)
{
    return (low <= x && x <= high);
}


//get measures of blocks given the size of aruco that is being read
int get_topmargin(int size_aruco) {
    return size_aruco * (0.3 / 0.8);
}


int get_bottommargin(int size_aruco) {
    return size_aruco * (0.2 / 0.8);
}


int get_topblock(int size_aruco) {
    return size_aruco * (1 / 0.8) * 0.15;
}


//check what function is associated with the detected aruco
void check_function(int id, int x, int y, cv::InputOutputArray image) {
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
    if (id == 0 || id == 1 || id == 3 || id == 4 || id == 5 || id == 6 || id == 13 || id == 25 || id == 26)
    {
        //2 inputs 1 output
        circle(image, cv::Point(sup_esq.x, sup_esq.y+((inf_esq.y-sup_esq.y)*(0.20))), 9, CV_RGB(0, 0, 255), 1.5); //input top
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75))), 9, CV_RGB(0, 0, 255), 1.5);//input bottom
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output

    }
    else if (id == 2 || id == 14 || id == 15 || id == 16 || id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22 || id == 23 || id == 24)
    {
        //1 input 1 output
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.20))), 9, CV_RGB(0, 0, 255), 1.5); //input top
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
    }
    else if (id == 7 || id == 8)
    {
        //1 input
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.20))), 9, CV_RGB(0, 0, 255), 1.5); //input top
    }
    else if (id == 9 || id == 10 || id == 11 || id == 12)
    {
        //1 output
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
    }
    else if (id == 27)
    {
        //2 inputs 1 output 1 condition
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.20))), 9, CV_RGB(0, 0, 255), 1.5); //input top
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75))), 9, CV_RGB(0, 0, 255), 1.5);//input bottom
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
        //circle(image, Point(inf_dir.y, inf_esq.x + ((inf_dir.x - inf_esq.x) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //condition
    }

}


void draw_block(bool check, cv::InputOutputArray image_camera, cv::InputOutputArray image_skeleton, int id, int pos, std::vector<std::vector<cv::Point2f>> corners)
{
    if ((check == true) && (id != 28) && (id != 29) && (id != 30))
    {
        size_aruco = corners[pos][1].x - corners[pos][0].x;
        block_i.b_sup_left.x = corners[pos][0].x - get_topblock(size_aruco);
        block_i.b_sup_left.y = corners[pos][0].y - get_topmargin(size_aruco);

        block_i.b_sup_right.x = corners[pos][1].x + get_topblock(size_aruco);
        block_i.b_sup_right.y = corners[pos][1].y - get_topmargin(size_aruco);

        block_i.b_inf_left.x = corners[pos][3].x - get_topblock(size_aruco);
        block_i.b_inf_left.y = corners[pos][3].y + get_bottommargin(size_aruco);

        block_i.b_inf_right.x = corners[pos][2].x + get_topblock(size_aruco);
        block_i.b_inf_right.y = corners[pos][2].y + get_bottommargin(size_aruco);


        line(image_camera, cv::Point(block_i.b_sup_left.x, block_i.b_sup_left.y), cv::Point(block_i.b_sup_right.x, block_i.b_sup_right.y), cv::Scalar(255), 2, 8, 0); //topo
        line(image_skeleton, cv::Point(block_i.b_sup_left.x, block_i.b_sup_left.y), cv::Point(block_i.b_sup_right.x, block_i.b_sup_right.y), cv::Scalar(255), 2, 8, 0); //topo

        line(image_camera, cv::Point(block_i.b_sup_left.x, block_i.b_sup_left.y), cv::Point(block_i.b_inf_left.x, block_i.b_inf_left.y), cv::Scalar(255), 2, 8, 0); //left
        line(image_skeleton, cv::Point(block_i.b_sup_left.x, block_i.b_sup_left.y), cv::Point(block_i.b_inf_left.x, block_i.b_inf_left.y), cv::Scalar(255), 2, 8, 0); //left

        line(image_camera, cv::Point(block_i.b_inf_left.x, block_i.b_inf_left.y), cv::Point(block_i.b_inf_right.x, block_i.b_inf_right.y), cv::Scalar(255), 2, 8, 0); //bottom
        line(image_skeleton, cv::Point(block_i.b_inf_left.x, block_i.b_inf_left.y), cv::Point(block_i.b_inf_right.x, block_i.b_inf_right.y), cv::Scalar(255), 2, 8, 0); //bottom

        line(image_camera, cv::Point(block_i.b_inf_right.x, block_i.b_inf_right.y), cv::Point(block_i.b_sup_right.x, block_i.b_sup_right.y), cv::Scalar(255), 2, 8, 0); //right
        line(image_skeleton, cv::Point(block_i.b_inf_right.x, block_i.b_inf_right.y), cv::Point(block_i.b_sup_right.x, block_i.b_sup_right.y), cv::Scalar(255), 2, 8, 0); //right


        check_function(id, block_i.b_sup_left.x, block_i.b_sup_left.y - 5, image_camera);
        check_function(id, block_i.b_sup_left.x, block_i.b_sup_left.y - 5, image_skeleton);


        draw_points(id, block_i.b_sup_left, block_i.b_sup_right, block_i.b_inf_left, block_i.b_inf_right, image_camera);
        draw_points(id, block_i.b_sup_left, block_i.b_sup_right, block_i.b_inf_left, block_i.b_inf_right, image_skeleton);
    }
}


void drawing_functions(cv::InputOutputArray image_camera, cv::InputOutputArray image_skeleton, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids) {
    if (ids.size() > 0)
    {
        current_ids_size = ids.size();
        cv::aruco::drawDetectedMarkers(image_camera, corners, ids);

        //run through every detected aruco
        for (int i = 0; i < corners.size(); i++)
        {

            orientation(ids[i], corners[i][0].x, corners[i][0].y);

            inRange(o_inf_left.x - 15, o_inf_left.x + 15, o_sup_left.x) && inRange(o_inf_right.y - 15, o_inf_right.y + 15, o_inf_left.y) && (o_sup_left.y < o_inf_left.y) ? orientation_check = true : orientation_check = false;

            draw_block(orientation_check, image_camera, image_skeleton, ids[i], i, corners);

            //cout << orientation_check;

            //if sheet is in the right orientation start analysis and drawing skeleton
        }
    }
}


void verify_o_arucos_in_sheet(std::vector<int> ids)
{
    if (ids.size() < current_ids_size)
    {
        if (!count(ids.begin(), ids.end(), 28))
        {
            o_sup_left.x = 0;
            o_sup_left.y = 0;
        }
        if (!count(ids.begin(), ids.end(), 29))
        {
            o_inf_left.x = 0;
            o_inf_left.y = 0;
        }
        if (!count(ids.begin(), ids.end(), 30))
        {
            o_inf_right.x = 0;
            o_inf_right.y = 0;
        }
    }
}

void aruco_mainfunction(cv::Mat frame, cv::Ptr<cv::aruco::Dictionary> dict) {
    cv::Mat frameCopy;
    cv::Mat skeleton(500, 900, CV_8UC3, cv::Scalar(255, 255, 255)); //blank image to draw skeleton

    frame.copyTo(frameCopy);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(frame, dict, corners, ids);

    //erase coordinates of orientation arucos if they are not detected in the sheet
    verify_o_arucos_in_sheet(ids);

    //drawing functions
    drawing_functions(frameCopy, skeleton, corners, ids);


    //cout << o_sup_left.x << " | " << o_sup_left.y << " || " << o_inf_left.x << " | " << o_inf_left.y << " || " << o_inf_right.x << " | " << o_inf_right.y ;
    //cout << "\n";


    imshow("out", frameCopy);
    char key = (char)cv::waitKey(30);
//    if (key == 27)
//        break;
    if (key == 's') //save blank image with skeleton by pressing "s" key
        imwrite("my_image.png", skeleton);
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
      cv::VideoCapture cap("../catkin_ws/src/SERP/serp/include/tests/2x2cm.h264");

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
        aruco_mainfunction(frame, dictionary);

        ros::spinOnce();
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}
