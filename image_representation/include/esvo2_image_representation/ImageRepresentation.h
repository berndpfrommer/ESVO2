#ifndef image_representation_H_
#define image_representation_H_

#include <cv_bridge/cv_bridge.hpp>
#include <esvo2_image_representation/Event.h>
#include <esvo2_image_representation/TicToc.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <event_camera_codecs/decoder.h>
#include <event_camera_msgs/msg/event_packet.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace esvo2_image_representation {
using EventPacket = event_camera_msgs::msg::EventPacket;
using EventPacketPtr = event_camera_msgs::msg::EventPacket::ConstSharedPtr;
using EventQueue = std::deque<Event>;
using Image = sensor_msgs::msg::Image;
using ImagePtr = Image::ConstSharedPtr;
using CameraInfo = sensor_msgs::msg::CameraInfo;

using GlobalEventQueue = std::map<timestamp_t, Event>;

inline static EventQueue::iterator EventBuffer_lower_bound(EventQueue &eb,
                                                           timestamp_t t) {
  return std::lower_bound(
      eb.begin(), eb.end(), t,
      [](const Event &e, timestamp_t t) { return e.ts < t; });
}

inline static EventQueue::iterator EventBuffer_upper_bound(EventQueue &eb,
                                                           timestamp_t t) {
  return std::upper_bound(
      eb.begin(), eb.end(), t,
      [](timestamp_t t, const Event &e) { return t < e.ts; });
}

inline static std::vector<Event>::iterator
EventVector_lower_bound(std::vector<Event> &ev, timestamp_t t) {
  return std::lower_bound(
      ev.begin(), ev.end(), t,
      [](const Event &e, timestamp_t t) { return e.ts < t; });
}

class EventHandler;

class ImageRepresentation : public rclcpp::Node {
public:
  explicit ImageRepresentation(const rclcpp::NodeOptions &options);
  virtual ~ImageRepresentation();

  static bool compare_time(const Event &e, const timestamp_t reference_time) {
    return reference_time < e.ts;
  }

private:
  // core
  void init(int width, int height);
  // Support: TS, AA, negative_TS, negative_TS_dx, negative_TS_dy
  void createImageRepresentationAtTime(const timestamp_t external_sync_time);
  void GenerationLoop();

  // callbacks
  void eventsCallback(const EventPacketPtr &msg);

  // utils
  void clearEventQueue();
  bool loadCalibInfo(const std::string &cameraSystemDir, bool &is_left);
  void clearEvents(int distance, std::vector<Event>::iterator ptr_e);

  void AA_thread(std::vector<Event>::iterator &ptr_e, int distance,
                 const timestamp_t external_t);
  void sobel(double external_t);
  bool fileExists(const std::string &filename);
  // tests

  // calibration parameters
  cv::Mat camera_matrix_, dist_coeffs_;
  cv::Mat rectification_matrix_, projection_matrix_;
  std::string distortion_model_;
  cv::Mat undistort_map1_, undistort_map2_;
  Eigen::Matrix2Xd precomputed_rectified_points_;

  // sub & pub
  rclcpp::Subscription<EventPacket>::SharedPtr event_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;

  image_transport::Publisher dx_image_pub_, dy_image_pub_;
  image_transport::Publisher image_representation_pub_TS_;
  image_transport::Publisher image_representation_pub_negative_TS_;
  image_transport::Publisher image_representation_pub_AA_frequency_;
  image_transport::Publisher image_representation_pub_AA_mat_;

  bool left_;
  cv::Mat negative_TS_img;
  cv_bridge::CvImage cv_dx_image, cv_dy_image;
  std::thread thread_sobel;

  // online parameters
  bool bCamInfoAvailable_;
  bool bUse_Sim_Time_;
  cv::Size sensor_size_;
  timestamp_t sync_time_{0};
  bool bSensorInitialized_;

  // offline parameters TODO
  double decay_ms_;
  bool ignore_polarity_;
  int median_blur_kernel_size_;
  int blur_size_;
  int max_event_queue_length_;
  int events_maintained_size_;

  // containers
  EventQueue events_;

  std::vector<Event> vEvents_;

  cv::Mat representation_TS_;
  cv::Mat representation_AA_;

  Eigen::MatrixXd TS_temp_map_;

  // for rectify
  cv::Mat undistmap1_, undistmap2_;
  bool is_left_, bcreat_;

  // thread mutex
  std::mutex data_mutex_;

  enum RepresentationMode {
    Linear_TS, // 0
    AA2,       // 1
    Fast       // 2
  } representation_mode_;

  // parameters
  bool bUseStereoCam_;
  double decay_sec_; // TS param
  int generation_rate_hz_;
  int x_patches_, y_patches_;
  // std::vector<dvs_msgs::Event>::iterator ptr_e_;

  // calib info
  std::string calibInfoDir_;
  std::vector<cv::Point> trapezoid_;
  std::shared_ptr<EventHandler> event_handler_;
};
} // namespace esvo2_image_representation
#endif // image_representation_H_