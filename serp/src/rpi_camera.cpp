#include "rpi_camera.h"

bool cb_read_programming_sheet(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    camera_state = ReadProgrammingSheet;
    res.success = true;
    return true;
}

void cb_robot_state(const std_msgs::StringConstPtr &state) {
    if(state->data == "Stopped" || state->data == "ManualControl") camera_state = NormalOperation;
    else if(state->data == "Executing") camera_state = DetectObstacles;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rpi_camera_node");
    ros::NodeHandle n_public;

    cv_bridge::CvImage img_bridge;

    // Create ROS Server
    ros::ServiceServer service_read_programming_sheet = n_public.advertiseService("srv_read_programming_sheet", cb_read_programming_sheet);

    // Image publishers
    image_transport::ImageTransport it(n_public);
    image_transport::Publisher pub_img = it.advertise("camera", 1);
    image_transport::Publisher pub_last_detected_sheet = it.advertise("camera/sheet_detections", 1);

    // Subscriber for Robot State
    ros::Subscriber sub_robot_state = n_public.subscribe("robot_state", 2, cb_robot_state);

    // Initialize camera state
    camera_state = NormalOperation;

    cv::VideoCapture cap = cv::VideoCapture("/home/jorge/Downloads/object_detection.mp4");
    if(!cap.isOpened()) {
        printf("Não foi possível abrir a câmara");
        return -1;
    }

    cv::Mat frame;
    while(ros::ok())
    {
        // Get new frame
        cap >> frame;
        //cv::imshow("Image", frame);
        cv::waitKey(100);

        cv::Mat cropped_image = frame(cv::Rect(100, 0, 1300, 1200));

//        // Resize image to 800x450 (to publish to the GUI)
        cv::Mat resized_frame;
        cv::resize(cropped_image, resized_frame, cv::Size(500, 462));


        // Convert OpenCV resized image to ROS data
        sensor_msgs::Image img_msg;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, resized_frame);
        img_bridge.toImageMsg(img_msg);
        // Publish image
        pub_img.publish(img_msg);
        ros::Duration(0.1).sleep();
        // Send image with detected blocks to the interface, for user checking
        if(camera_state == ReadProgrammingSheet) {
            pub_last_detected_sheet.publish(img_msg);
            camera_state = NormalOperation;
        }


        ros::spinOnce();
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
