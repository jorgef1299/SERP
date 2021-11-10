#include "rpi_camera.h"

void cbNewTask(const serp::Task::ConstPtr& msg) {

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rpi_camera_node");
    ros::NodeHandle n_public;
    // Create Subscriber to the tasks topic
    ros::Subscriber sub = n_public.subscribe("/tasks", 1, cbNewTask);
    // Publishers
    ros::Publisher Laser_pub = n_public.advertise<serp::LaserScanAdapted>("/laser_scan", 1);
//    ros::Publisher Laser_pub = n_public.advertise<serp::LaserScanAdapted>("/laser_scan", 1); //TODO: Add pub for line follower mode

    cv::VideoCapture cap("/dev/video0");
    if(!cap.isOpened()) {
        ROS_ERROR("Can't open Raspberry Pi camera!");
        return -1;
    }
    cv::Mat frame;
    while(ros::ok())
    {
        // Get new frame
        if(!cap.read(frame)) // Camera has been disconnected
        {
            ROS_ERROR("Can't read Raspberry Pi camera! Please check the connections...");
            break;
        }

        // Show new frame
        cv::namedWindow("RPi Camera Frame");
        cv::imshow("RPi Camera Frame", frame);
        cv::waitKey(30);

        // TODO: Process frame if needed for a task
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
