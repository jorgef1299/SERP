#include "video_tests.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_tests_node");
    ros::NodeHandle n_public;

    // Create a VideoCapture object and open the input file
      // If the input is the web camera, pass 0 instead of the video file name
      cv::VideoCapture cap("../catkin_ws/src/SERP/serp/include/tests/random_video.h264");

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
        cv::imshow("Frame", frame);

        // Press  ESC on keyboard to exit
        char c=(char)cv::waitKey(25);
        if(c==27) break;

        ros::spinOnce();
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}
