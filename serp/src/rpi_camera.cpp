#include "rpi_camera.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rpi_camera_node");
    ros::NodeHandle n_public;

    cv_bridge::CvImage img_bridge;

    // Image publisher
    image_transport::ImageTransport it(n_public);
    image_transport::Publisher pub_img = it.advertise("camera", 1);

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

//        // Resize image to 800x450 (to publish to the GUI)
//        cv::Mat resized_frame;
//        cv::resize(frame, resized_frame, cv::Size(400, 225));

        // Convert OpenCV resized image to ROS data
        sensor_msgs::Image img_msg;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        img_bridge.toImageMsg(img_msg);
        // Publish image
        pub_img.publish(img_msg);

        /*
        // Show new frame
        cv::namedWindow("RPi Camera Frame");
        cv::imshow("RPi Camera Frame", frame);
        cv::waitKey(30);
        */
        ros::spinOnce();
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
