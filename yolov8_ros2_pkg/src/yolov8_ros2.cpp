#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include "std_msgs/msg/int32.hpp"
#include <filesystem>
#include "yolov8_ros2_pkg/utils.h"
#include "yolov8_ros2_pkg/yolov8Predictor.h"

class objectDetectionPublish : public rclcpp::Node
{
public:
  objectDetectionPublish() : Node("object_detection_publish")
  {
    // Konfigurasi YOLO
    float confThreshold = 0.4f;
    float iouThreshold = 0.4f;
    float maskThreshold = 0.5f;
    bool isGPU = false;
    // bool is_camera_open = true;

    std::string modelPath = "/home/m/linorobot2_ws/src/linorobot2/yolov8_ros2_pkg/models/bball.onnx";
    std::string classNamesPath = "/home/m/linorobot2_ws/src/linorobot2/yolov8_ros2_pkg/models/bball.names";

    classNames = utils::loadNames(classNamesPath);
    if (classNames.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "File class name kosong!");
      rclcpp::shutdown();
      return;
    }

    if (!std::filesystem::exists(modelPath))
    {
      RCLCPP_ERROR(this->get_logger(), "Model YOLO tidak ditemukan di path: %s", modelPath.c_str());
      rclcpp::shutdown();
      return;
    }

    try
    {
      predictor = std::make_unique<YOLOPredictor>(modelPath, isGPU, confThreshold, iouThreshold, maskThreshold);
      RCLCPP_INFO(this->get_logger(), "Model YOLO berhasil diinisialisasi.");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Gagal inisialisasi model: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    assert(classNames.size() == static_cast<size_t>(predictor->classNums));

    // Konfigurasi RealSense
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280,720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280,720, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    width = 1280;
    height = 720;
    fps = 30;

    // Publisher
    koordinat_pub = this->create_publisher<geometry_msgs::msg::Point>("coordinate_object", 10); //koodinat yolo
    boundingBox_pub = this->create_publisher<geometry_msgs::msg::Point>("coordinate_boundingBox", 10); //koordinat hsv
    button_subs_ = this-> create_subscription<std_msgs::msg::Int32>("triangle",10,std::bind(&objectDetectionPublish::triangle_callback, this,std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&objectDetectionPublish::processFrame, this));
  }

  ~objectDetectionPublish()
  {
    pipe.stop();
    cv::destroyAllWindows();
  }

private:

  bool is_camera_open = true;

  void triangle_callback(const std_msgs::msg::Int32::SharedPtr msg){


    if (msg->data == 1){

      if(is_camera_open)
      {
        is_camera_open = false;
        cv::destroyAllWindows();
        RCLCPP_INFO(this->get_logger(), "KAMERA DI TUTUP");
      }
      else{
        is_camera_open = true;
        RCLCPP_INFO(this->get_logger(), "KAMERA DIBUKA");
      }
    }

  }

  void processFrame()
  {

    if (!is_camera_open)return;


    rs2::frameset frameset;
    if (!pipe.poll_for_frames(&frameset))
    {
      RCLCPP_WARN(this->get_logger(), "Frame RealSense belum tersedia.");
      return;
    }

    rs2::video_frame color_frame = frameset.get_color_frame();
    rs2::depth_frame depth_frame = frameset.get_depth_frame();

    if (!color_frame || !depth_frame)
    {
      RCLCPP_WARN(this->get_logger(), "Frame warna tidak valid.");
      return;
    }

    cv::Mat frame(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat depth_raw(cv::Size(width, height), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

    cv::Mat depth_display;
    depth_raw.convertTo(depth_display, CV_8UC1, 15.0 / 10000);
    cv::applyColorMap(depth_display, depth_display, cv::COLORMAP_JET);

    if (frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Frame kosong.");
      return;
    }

    cv::Point yolo_center(-1, -1); // Titik tengah dari deteksi YOLO
    cv::Point hsv_center(-1, -1);  // Titik HSV terdekat
    cv::Point last_yolo_center(-1, -1);
    cv::Point closestBoundingBox(-1, -1);

    double min_distance = std::numeric_limits<double>::max();

    // Proses deteksi YOLO
    auto result = predictor->predict(frame);

    // bool yolo_detected = false;
    if (!result.empty())
    {
      for (const auto &detection : result)
      {
        if (detection.classId != 0)
          continue;

        float centerX = detection.box.x + detection.box.width / 2;
        float centerY = detection.box.y + detection.box.height / 2;

        yolo_center = cv::Point(centerX, centerY); // Simpan ke variabel global
        last_yolo_center = yolo_center;

        cv::circle(frame, yolo_center, 5, cv::Scalar(255, 255, 0), 2);
        cv::line(frame, yolo_center, cv::Point(width / 2, height / 2), cv::Scalar(255, 255, 0), 2);

        geometry_msgs::msg::Point coordinates;
        coordinates.x = centerX;
        coordinates.y = centerY;
        coordinates.z = detection.conf;
        koordinat_pub->publish(coordinates);

        // yolo_detected = true;

        // break; // Bisa kamu pakai jika hanya satu objek
      }
    }

    // Deteksi warna merah (HSV)
    cv::Mat hsv_frame;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, red_mask;
    // cv::inRange(hsv_frame, cv::Scalar(0, 120, 70), cv::Scalar(3, 255, 255), mask1);
    // cv::inRange(hsv_frame, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    cv::inRange(hsv_frame, cv::Scalar(0, 100, 100), cv::Scalar(3, 255, 255), mask1);
    cv::inRange(hsv_frame, cv::Scalar(170, 93, 93), cv::Scalar(180, 255, 255), mask2);
    red_mask = mask1 | mask2;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // cv::Point reference_point = (yolo_detected && yolo_center.x >= 0) ? yolo_center : last_yolo_center;

    for (const auto &contour : contours)
    {
      if (cv::contourArea(contour) > 500)
      {
        cv::Rect boundingBox = cv::boundingRect(contour);
        cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 0), 2);


        cv::Point current_center = (boundingBox.tl() + boundingBox.br()) / 2;
        cv::circle(frame, current_center, 5, cv::Scalar(0, 255, 255), -1);

        // Hitung jarak ke titik yolo_center
        if (yolo_center.x >= 0 && yolo_center.y >= 0)
        {
          double distance = std::hypot(yolo_center.x - current_center.x, yolo_center.y - current_center.y);

          std::string distanceText = "Line froms YOLO to HSV :" + std::to_string(static_cast<int>(distance));
          cv::putText(frame, distanceText, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

          if (distance < min_distance)
          {
            min_distance = distance;
            hsv_center = current_center; // Simpan yang terdekat
            closestBoundingBox = cv::Point (boundingBox.x, boundingBox.y);
          }
        }
      }
    }
    if (yolo_center.x >= 0 && hsv_center.x >= 0 && closestBoundingBox.x >= 0)
    // if (reference_point.x >= 0 && hsv_center.x >= 0 && closestBoundingBox.x >= 0)

    {
      cv::line(frame, yolo_center, hsv_center, cv::Scalar(255, 0, 0), 2);
      // cv::line(frame, reference_point, hsv_center, cv::Scalar(255, 0, 0), 2);


      cv::Point textOrg(closestBoundingBox.x, closestBoundingBox.y - 10);
      cv::putText(frame, "ID 1", textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,255), 2);
      // RCLCPP_INFO(this->get_logger(), "ID 1"); 
  
      geometry_msgs::msg::Point coordinatesBB;
      coordinatesBB.x = hsv_center.x;
      coordinatesBB.y = hsv_center.y;
      coordinatesBB.z = 0.0f;
      boundingBox_pub->publish(coordinatesBB);
    }
    

    // Visualisasi akhir
    utils::visualizeDetection(frame, result, classNames);
    cv::line(frame, cv::Point(0, height / 2), cv::Point(width, height / 2), cv::Scalar(0, 0, 0), 2);
    cv::line(frame, cv::Point(width / 2, 0), cv::Point(width / 2, height), cv::Scalar(0, 0, 0), 2);
    cv::circle(frame, cv::Point(width / 2, height / 2), 5, cv::Scalar(0, 255, 0), -1);

    cv::imshow("YOLOv8 & HSV Detection", frame);
    // cv::imshow("YOLOV8 & HSV DETECTION", depth_display);
    if (cv::waitKey(1) == 'q')
    {
      rclcpp::shutdown();
    }
  }

  rs2::pipeline pipe;
  int width, height, fps;

  std::unique_ptr<YOLOPredictor> predictor;
  std::vector<std::string> classNames;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr koordinat_pub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr boundingBox_pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr button_subs_;


  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<objectDetectionPublish>());
  rclcpp::shutdown();
  return 0;
}
