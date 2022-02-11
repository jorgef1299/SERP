#include "rpi_camera.h"
using std::string;

bool cb_read_programming_sheet(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    camera_state = ReadProgrammingSheet;
    res.success = true;
    return true;
  {
      case 'f': //frente
          path.append("frente");
          break;

      case 't': //trás
          path.append("tras");
          break;

      case 'd': //direita
          path.append("direita");
          break;

      case 'e': //esquerda
          path.append("esquerda");
          break;
  }

  switch (valor_sensor)
  {
      case 1:
          path.append("1.png");
          break;
      case 2:
          path.append("2.png");
          break;
      case 3:
          path.append("3.png");
          break;
      case 4:
          path.append("4.png");
          break;

  }
  //ROS_INFO_STREAM("file_path: " << path);
  return path;
}

cv::Mat place_sensor(cv::Mat frame, cv::Mat sensor, int position_x, int position_y){

  cv::Mat mask;
  cv::Mat rgbLayer[4];
  cv::split(sensor,rgbLayer);

  if(sensor.channels() == 4)
      {
          split(sensor,rgbLayer);         // seperate channels
          cv::Mat cs[3] = { rgbLayer[0],rgbLayer[1],rgbLayer[2] };
          merge(cs,3,sensor);  // glue together again
          mask = rgbLayer[3];       // png's alpha channel used as mask
      }

  // Get the destination ROI (and make sure it is within the image)
  cv::Rect dstRC = cv::Rect(position_x, position_y, sensor.size().width, sensor.size().height);
  cv::Mat dstROI = frame(dstRC);
  // Copy the pixels from src to dst.
  sensor.copyTo(dstROI,mask);

  return frame;
}

cv::Mat process_frame(cv::Mat frame)
{
  double angle = 90;

  //Rotate frame 90º
   cv::Point2f center((frame.cols - 1) / 2.0, (frame.rows - 1) / 2.0);
   cv::Mat rotation_matix = getRotationMatrix2D(center, angle, 1.0);
   cv::warpAffine(frame, frame, rotation_matix, frame.size());

   // Resize video frame (to publish to the GUI)
   cv::resize(frame, frame, cv::Size(400, 400));


   for(;;)
   {

     // Import sensor images
     cv::Mat sensor_front = cv::imread(find_sensorIMG_path('f',2), cv::IMREAD_UNCHANGED);
     cv::Mat sensor_back = cv::imread(find_sensorIMG_path('t',1), cv::IMREAD_UNCHANGED);
     cv::Mat sensor_left = cv::imread(find_sensorIMG_path('e',3), cv::IMREAD_UNCHANGED);
     cv::Mat sensor_right = cv::imread(find_sensorIMG_path('d',4), cv::IMREAD_UNCHANGED);

     //Resize sensor images
     cv::resize(sensor_front,sensor_front,cv::Size(60,37));
     cv::resize(sensor_back,sensor_back,cv::Size(60,37));
     cv::resize(sensor_left,sensor_left,cv::Size(60,71));
     cv::resize(sensor_right,sensor_right,cv::Size(60,71));

     int cx = (frame.cols - sensor_front.size().width) / 2;
     if (!sensor_front.empty()) {
         frame = place_sensor(frame, sensor_front, cx, frame.rows/7);
     }
     if (!sensor_back.empty()) {
         frame = place_sensor(frame, sensor_back, cx, 5.2*frame.rows/7);

     }
     if (!sensor_left.empty()) {
         frame = place_sensor(frame, sensor_left, 7*cx/12, frame.rows/6);
     }
     if (!sensor_right.empty()) {
         frame = place_sensor(frame, sensor_right, 17*cx/12, frame.rows/6);
     }

     return frame;
   }

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
