#include "rpi_camera.h"
#include "objDetect.h"

std::vector<uint8_t> sensor_valores;

bool cb_camera_control(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
{
    if (req.data == true)
    {
        // Publish camera data
        must_publish_camera_data = true;
    }
    else
    {
        must_publish_camera_data = false;
    }
    // Send response
    res.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpi_camera_node");
    ros::NodeHandle n_public;
    serp::Sensors sensors_data;
    ros::Publisher send_sensors = n_public.advertise<serp::Sensors>("send_sensors", 1);

    // Initialize variable(s)
    must_publish_camera_data = true;
    cv_bridge::CvImage img_bridge;

    // Image publisher
    image_transport::ImageTransport it(n_public);
    image_transport::Publisher pub_img = it.advertise("camera", 1);

    // Create ROS Servers
    ros::ServiceServer srv_camera = n_public.advertiseService("set_camera", cb_camera_control);

    raspicam::RaspiCam_Cv Camera;
    Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    Camera.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);

    std::cout << "Opening Camera..." << std::endl;
    if (!Camera.open())
    {
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }
    ros::Duration(1.0).sleep();
    cv::Mat frame;

    while (ros::ok())
    {
        // Get new frame
        Camera.grab();
        Camera.retrieve(frame);
        //process object detection
        sensor_valores = frameProcessing(frame);
	
        //fazer cena de nao entupir
    	sensors_data.right=sensor_valores[0];
    	sensors_data.front=sensor_valores[1];
    	sensors_data.back=sensor_valores[2];
    	sensors_data.left=sensor_valores[3];
            send_sensors.publish(sensors_data);
          
        // Publish image (if needed)
        if (must_publish_camera_data)
        {
            // Convert OpenCV image to ROS data
            sensor_msgs::Image img_msg;
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
            img_bridge.toImageMsg(img_msg);
            pub_img.publish(img_msg);
        }
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
