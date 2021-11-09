#include "usb_camera.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "usb_camera_node");
    ros::NodeHandle n_private("~");
//    ros::Publisher pub = n_private.advertise<>("/tasks", 1); //TODO: Add message type

    cv::VideoCapture cap("/dev/video0");
    if(!cap.isOpened()) {
        ROS_ERROR("Can't open usb camera!");
        std::exit(-1);
    }
    cv::Mat frame;
    while(ros::ok())
    {
        // Get new frame
        if(!cap.read(frame)) // Camera has been disconnected
        {
            ROS_ERROR("Can't read usb camera! Please check the connections...");
            break;
        }

        // Show new frame
        cv::namedWindow("USB Camera Frame");
        cv::imshow("USB Camera Frame", frame);
        cv::waitKey(30);

        // TODO: Process the received frame to see if there is any programming sheet in the image
        // Note: If there isn't movement (since the last frame we processed), probably the programming sheet is not in the image
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}


void processFrame(cv::Mat image)
{

}