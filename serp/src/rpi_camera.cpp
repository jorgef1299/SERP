#include "rpi_camera.h"
#include <raspicam/raspicam_cv.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rpi_camera_node");
    ros::NodeHandle n_public;

    cv_bridge::CvImage img_bridge;

    // Image publisher
    image_transport::ImageTransport it(n_public);
    image_transport::Publisher pub_img = it.advertise("camera", 1);

    raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
    Camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    Camera.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
    //Open camera
    std::cout<<"Opening Camera..."<<std::endl;
    if (!Camera.open()) {std::cerr<<"Error opening the camera"<<std::endl;return -1;}
    ros::Duration(1.0).sleep();

    cv::Mat frame;
    while(ros::ok())
    {
        // Get new frame
        Camera.grab();
        Camera.retrieve(frame);
        //cv::imshow("Image", frame);
        cv::waitKey(1000);


//        // Resize image to 800x450 (to publish to the GUI)
        cv::Mat resized_frame;
        cv::resize(frame, resized_frame, cv::Size(400, 225));

        // Convert OpenCV resized image to ROS data
        sensor_msgs::Image img_msg;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, resized_frame);
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
    Camera.release();
    cv::destroyAllWindows();
    return 0;
}
