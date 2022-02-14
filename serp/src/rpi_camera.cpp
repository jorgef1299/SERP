#include "rpi_camera.h"
int ready;
cv::Mat final;





std::vector<uint8_t> sensor_valores;

bool cb_read_programming_sheet(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    camera_state = ReadProgrammingSheet;
    res.success = true;
    return true;
}

void cb_robot_state(const std_msgs::StringConstPtr &state) {
    if(state->data == "Stopped" || state->data == "ManualControl") camera_state = NormalOperation;
    else if(state->data == "Executing") camera_state = DetectObstacles;
}

int main(int argc, char **argv)
{


    
    int count=0, j=0;
    ros::init(argc, argv, "rpi_camera_node");
    ros::NodeHandle n_public;



    // Subscriber for Robot State
    ros::Subscriber sub_robot_state = n_public.subscribe("robot_state", 2, cb_robot_state);
    camera_state=NormalOperation;

    // Initialize variable(s)

    cv_bridge::CvImage img_bridge;
    //aruco dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    uint8_t pleft;
    uint8_t pright;
    uint8_t pback;
    uint8_t pfront;
    int matrix_size=100;
    int matrix_links[100][100]={0};
    float matrix_values[100][100]={0};


    // Matrixes publisher
    serp::Matrix data;
    ros::Publisher send_matrix = n_public.advertise<serp::Matrix>("send_matrix", 1);


    // Image publisher
    image_transport::ImageTransport it(n_public);
    image_transport::Publisher pub_img = it.advertise("camera", 1);
    image_transport::Publisher pub_last_detected_sheet = it.advertise("camera/sheet_detections", 1); //usar para mandar frame

    // Create ROS Servers
    ros::ServiceServer service_read_programming_sheet = n_public.advertiseService("srv_read_programming_sheet", cb_read_programming_sheet);

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

        cv::Mat cropped_image = frame(cv::Rect(100, 0, 1300, 1200));

        //        // Resize image to 800x450 (to publish to the GUI)
        cv::Mat resized_frame;
        cv::resize(cropped_image, resized_frame, cv::Size(500, 462));


        // Publish image (if needed)
        if(count == 0)
        {
            // Convert OpenCV image to ROS data
            sensor_msgs::Image img_msg;
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, resized_frame);
            img_bridge.toImageMsg(img_msg);
            pub_img.publish(img_msg);
        }

        if(camera_state == DetectObstacles)
        {

            sensor_valores = frameProcessing(frame);

            //fill matrix_values with sensor values
            for(int i=0; i < matrix_size; i++)
            {
                //if left sensor is connected to any block
                if(matrix_links[i][2] == 1)
                {
                    matrix_values[i][2] = sensor_valores[3];
                    matrix_values[2][i] = sensor_valores[3];
                }

                //if front sensor is connected to any block
                if(matrix_links[i][3] == 1)
                {
                    matrix_values[i][3] = sensor_valores[1];
                    matrix_values[3][i] = sensor_valores[1];
                }

                //if right sensor is connected to any block
                if(matrix_links[i][4] == 1)
                {
                    matrix_values[i][4] = sensor_valores[0];
                    matrix_values[4][i] = sensor_valores[0];
                }

                //if back sensor is connected to any block
                if(matrix_links[i][5] == 1)
                {
                    matrix_values[i][5] = sensor_valores[2];
                    matrix_values[5][i] = sensor_valores[2];
                }
            }

            //publish matrixes
            for (int i=0; i < matrix_size; i++)
            {
                for (int j=0 ; j< matrix_size ; j++)
                {
                    data.matrix_links.push_back(matrix_links[i][j]);
                    data.matrix_values.push_back(matrix_values[i][j]);
                }
            }

            send_matrix.publish(data);
            //falta sobrepor imagem com sensores (parte da Ana)
        }
        else if(camera_state == ReadProgrammingSheet)
        {
            ready=detectAndInterpret_Paper(frame, dictionary, matrix_links, matrix_values, ready);
            if(ready == 1)
            {
                // Convert OpenCV image to ROS data
                sensor_msgs::Image img_msg;
                std_msgs::Header header;
                header.stamp = ros::Time::now();
                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
                img_bridge.toImageMsg(img_msg);
                //publish image to interface
                pub_last_detected_sheet.publish(img_msg);

                //return to normal operation state until user decides
                camera_state = NormalOperation;
                ready=0;

            }
        }

        ros::spinOnce();
        count++;
        if (count==2)
            count=0;
    }
    Camera.release();
    cv::destroyAllWindows();
    return 0;
}
