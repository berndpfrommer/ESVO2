#ifndef ESVO2_CORE_MAPPING_H
#define ESVO2_CORE_MAPPING_H

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <events_repackaging_msgs/msg/vbabg.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <tf2/buffer_core.hpp>

#include <esvo2_core/core/timestamp.h>

#include <esvo2_core/container/CameraSystem.h>
#include <esvo2_core/container/DepthMap.h>
#include <esvo2_core/container/EventMatchPair.h>
#include <esvo2_core/core/DepthFusion.h>
#include <esvo2_core/core/DepthProblem.h>
#include <esvo2_core/core/DepthProblemSolver.h>
#include <esvo2_core/core/DepthRegularization.h>
#include <esvo2_core/core/EventBM.h>
#include <esvo2_core/tools/Visualization.h>
#include <esvo2_core/tools/utils.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <deque>
#include <future>
#include <map>
#include <mutex>

#include <cv_bridge/cv_bridge.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <esvo2_core/core/BackendOptimization.h>
#include <esvo2_core/factor/imu_integration.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <event_camera_codecs/event_processor.h>

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>

namespace esvo2_core {
using namespace core;

class EventHandler;

class esvo2_Mapping : public rclcpp::Node {
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PosePtr = PoseStamped::ConstSharedPtr;
  using Image = sensor_msgs::msg::Image;
  using ImagePtr = Image::ConstSharedPtr;
  using ImuMsg = sensor_msgs::msg::Imu;
  using ImuPtr = ImuMsg::ConstSharedPtr;
  using EventPacket = event_camera_msgs::msg::EventPacket;
  using EventPacketPtr = EventPacket::ConstSharedPtr;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2Ptr = PointCloud2::ConstSharedPtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  esvo2_Mapping(const rclcpp::NodeOptions &options);
  virtual ~esvo2_Mapping();

  // mapping
  void MappingLoop(std::promise<void> prom_mapping,
                   std::future<void> future_reset);
  void MappingAtTime(const timestamp_t t);
  bool InitializationAtTime(const timestamp_t t);
  bool dataTransferring();

  // callback functions
  void stampedPoseCallback(const PosePtr &ps_msg);
  void eventsCallback(const EventPacketPtr &msg);
  void timeSurfaceCallback(const ImagePtr &time_surface_left,
                           const ImagePtr &time_surface_right,
                           const ImagePtr &AA_map,
                           const ImagePtr &time_surface_negative,
                           const ImagePtr &time_surface_dx,
                           const ImagePtr &time_surface_dy);
  void AACallback(const ImagePtr &AA_left);
  void refImuCallback(const ImuPtr &msg);
  // utils
  bool getPoseAt(timestamp_t t, Transformation &Tr,
                 const std::string &source_frame);
  void clearEventQueue(EventQueue &EQ);
  void reset();

  /*** publish results ***/
  void publishMappingResults(DepthMap::Ptr depthMapPtr, Transformation tr,
                             timestamp_t t);
  void publishPointCloud(DepthMap::Ptr &depthMapPtr, Transformation &tr,
                         timestamp_t t);
  void publishImage(const cv::Mat &image, const timestamp_t t,
                    image_transport::Publisher &pub,
                    std::string encoding = "bgr8");

  /*** event processing ***/
  void
  createEdgeMask(std::vector<Event *> &vEventsPtr,
                 PerspectiveCamera::Ptr &camPtr, cv::Mat &edgeMap,
                 std::vector<std::pair<size_t, size_t>> &vEdgeletCoordinates,
                 bool bUndistortEvents = true, size_t radius = 0);

  void createDenoisingMask(std::vector<Event *> &vAllEventsPtr, cv::Mat &mask,
                           size_t row,
                           size_t col); // reserve in this file

  void extractDenoisedEvents(std::vector<Event *> &vCloseEventsPtr,
                             std::vector<Event *> &vEdgeEventsPtr,
                             cv::Mat &mask, size_t maxNum = 5000);

  void getReprojection(std::vector<EventMatchPair> &vEMP,
                       Eigen::Matrix4d T_last_now,
                       std::vector<Event *> &vDenoisedEventsPtr_left_dy_);
  void selectPoint();
  bool getIMUInterval(double t0, double t1,
                      vector<pair<double, Eigen::Vector3d>> &accVector,
                      vector<pair<double, Eigen::Vector3d>> &gyrVector);
  void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
  void processIMU(double t, double dt,
                  const Eigen::Vector3d &linear_acceleration,
                  const Eigen::Vector3d &angular_velocity);

  /************************ member variables ************************/
private:
  // Subscribers
  rclcpp::Subscription<EventPacket>::SharedPtr events_left_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr stamped_pose_sub_;
  rclcpp::Subscription<Image>::SharedPtr AA_frequency_sub_;

  message_filters::Subscriber<Image> TS_left_sub_, TS_right_sub_;
  message_filters::Subscriber<Image> AA_map_sub_;
  message_filters::Subscriber<Image> TS_negative_sub_, TS_dx_sub_, TS_dy_sub_;

  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<PointCloud2>::SharedPtr pc_pub_, gpc_pub_, pc_filtered_pub_;
  rclcpp::Publisher<events_repackaging_msgs::msg::Vbabg>::SharedPtr
      V_ba_bg_pub_;
  timestamp_t t_last_pub_pc_;

  // Time-Surface sync policy
  typedef message_filters::sync_policies::ExactTime<Image, Image>
      ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<Image, Image, Image,
                                                          Image, Image, Image>
      ApproxSyncPolicy2;
  typedef message_filters::sync_policies::ApproximateTime<Image, Image>
      ApproxSyncPolicy;
  message_filters::Synchronizer<ApproxSyncPolicy> TS_sync_;
  message_filters::Synchronizer<ApproxSyncPolicy2> TS_AA_sync_;

  // offline data
  std::string dvs_frame_id_;
  std::string world_frame_id_;
  std::string calibInfoDir_;
  CameraSystem::Ptr camSysPtr_;

  // imu data
  timestamp_t prev_time_;
  bool first_imu;

  // online data
  EventQueue events_left_, events_right_;
  TimeSurfaceHistory TS_history_;
  constStampedTimeSurfaceObs *TS_obs_ptr_;
  StampTransformationMap st_map_;
  std::shared_ptr<tf2::BufferCore> tf_;
  size_t TS_id_;
  timestamp_t tf_lastest_common_time_;

  // system
  std::string ESVO2_System_Status_;
  DepthProblemConfig::Ptr dpConfigPtr_, dpConfigPtr_ln_;
  DepthProblemSolver dpSolver_, dpSolver_ln_;
  DepthFusion dFusor_, dFusor_ln_;
  DepthRegularization dRegularizor_, dRegularizor_ln_;
  Visualization visualizor_;
  EventBM ebm_;

  // data transfer
  std::vector<Event *> vALLEventsPtr_left_;          // for BM
  std::vector<Event *> vCloseEventsPtr_left_;        // for BM
  std::vector<Event *> vDenoisedEventsPtr_left_;     // for BM
  std::vector<Event *> vDenoisedEventsPtr_left_dx_;  // for BM
  std::vector<Event *> vDenoisedEventsPtr_left_dx2_; // for BM
  std::vector<Event *> vDenoisedEventsPtr_left_dy_;  // for BM
  size_t totalNumCount_; // count the number of events involved
  std::vector<Event *> vEventsPtr_left_SGM_; // for SGM

  // result
  PointCloud::Ptr pc_near_, pc_global_;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc_color_, pc_filtered_;
  DepthFrame::Ptr depthFramePtr_;
  bool blarge_scale_, bpoints_from_AA_;

  // std::deque<std::vector<DepthPoint> > dqvDepthPoints_,dqvDepthPoints_ln_;
  std::deque<DepthPointFrame> dqvDepthPoints_, dqvDepthPoints_ln_;
  // DepthPointFrame

  // inter-thread management
  std::mutex data_mutex_;
  std::promise<void> mapping_thread_promise_, reset_promise_;
  std::future<void> mapping_thread_future_, reset_future_;

  /**** mapping parameters ***/
  // range and visualization threshold
  double invDepth_min_range_;
  double invDepth_max_range_;
  double cost_vis_threshold_, cost_vis_threshold_ln_;
  size_t patch_area_;
  double residual_vis_threshold_, residual_vis_threshold_ln_;
  double stdVar_vis_threshold_, stdVar_vis_threshold_ln_;
  size_t age_max_range_;
  size_t age_vis_threshold_;
  int fusion_radius_;
  std::string FusionStrategy_;
  int maxNumFusionFrames_, maxNumFusionFrames_ln_;
  int maxNumFusionPoints_;
  size_t INIT_SGM_DP_NUM_Threshold_;
  // module parameters
  size_t PROCESS_EVENT_NUM_;
  size_t PROCESS_EVENT_NUM_AA_;
  size_t TS_HISTORY_LENGTH_;
  size_t mapping_rate_hz_;
  // options
  bool bRegularization_;
  bool resetButton_;
  bool bDenoising_;
  bool bVisualizeGlobalPC_;
  // visualization parameters
  double visualizeGPC_interval_;
  double visualize_range_;
  size_t numAddedPC_threshold_;
  // Event Block Matching (BM) parameters
  timestamp_t BM_half_slice_thickness_;
  double eta_for_select_points_;
  size_t BM_patch_size_X_, BM_patch_size_X_2_;
  size_t BM_patch_size_Y_, BM_patch_size_Y_2_;
  size_t BM_min_disparity_;
  size_t BM_max_disparity_;
  size_t BM_step_;
  double BM_ZNCC_Threshold_;
  bool BM_bUpDownConfiguration_;
  bool bUSE_IMU_;
  // Select points from AA
  int x_patches_, y_patches_;

  double distance_from_last_frame_;

  // SGM parameters (Used by Initialization)
  int num_disparities_;
  int block_size_;
  int P1_;
  int P2_;
  int uniqueness_ratio_;
  cv::Ptr<cv::StereoSGBM> sgbm_;

  BackendOptimization BackendOpt_;

  queue<pair<double, Eigen::Vector3d>> accBuf;
  queue<pair<double, Eigen::Vector3d>> gyrBuf;
  bool initFirstPoseFlag;
  Eigen::Vector3d acc_0, gyr_0;
  std::mutex mBuf;

  std::shared_ptr<EventHandler> event_handler_;

  /**********************************************************/
  /******************** For test & debug ********************/
  /**********************************************************/

  image_transport::Publisher invDepthMap_pub_, stdVarMap_pub_, ageMap_pub_,
      costMap_pub_, invDepthMap_rel_pub_;
  std::string resultPath_;
  // For counting the total number of fusion
  size_t TotalNumFusion_;
  double data_trans_time;
};

class EventHandler : public event_camera_codecs::EventProcessor {
public:
  EventHandler(esvo2_Mapping *mapping, EventQueue *eq, uint16_t width,
               uint16_t height)
      : mapping_(mapping), event_queue_(eq), width_(width), height_(height) {}
  virtual ~EventHandler() {}

  // ------ EventProcessor interface overrides ------
  void finished() override { is_first_in_message_ = true; }

  void eventCD(uint64_t ts, uint16_t ex, uint16_t ey,
               uint8_t polarity) override {
    constexpr int64_t max_time_diff_before_reset_s = 500000000LL;
    if (is_first_in_message_) {
      is_first_in_message_ = false;
      //
      // check timestamp consistency only for first event
      //
      if (!event_queue_->empty()) {
        const timestamp_t dt = ts - event_queue_->back().ts;
        if (dt < 0 ||
            std::abs<timestamp_t>(dt) >= max_time_diff_before_reset_s) {
          RCLCPP_INFO(
              mapping_->get_logger(),
              "Inconsistent event timestamps detected <eventCallback> (new: "
              "%ld, old %ld), resetting.",
              ts, event_queue_->back().ts);
          mapping_->reset();
        }
      }
    }
    if (ex < width_ && ey < height_) {
      event_queue_->emplace_back(ts, ex, ey, polarity);
      if (event_queue_->size() > 1) {
        int i = event_queue_->size() - 2;
        if ((*event_queue_)[i].ts > ts) {
          // sort the queue to ensure chronological order
          // move elements that are later in time towards end of queue to make
          // space for the new element
          for (; (*event_queue_)[i].ts > ts && i >= 0; i--) {
            (*event_queue_)[i + 1] = (*event_queue_)[i];
          }
          (*event_queue_)[i + 1] = Event(ts, ex, ey, polarity);
        }
      }
    }
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }
  void rawData(const char *, size_t) override {};

  // ------------ own methods
  void handleMessage(const esvo2_Mapping::EventPacketPtr &msg) {
    if (!decoder_) {
      event_camera_codecs::DecoderFactory<esvo2_Mapping::EventPacket,
                                          EventHandler>
          decoderFactory;
      decoder_ = decoderFactory.getInstance(*msg);
      if (!decoder_) {
        RCLCPP_ERROR_STREAM(mapping_->get_logger(),
                            "Failed to create decoder instance for "
                                << msg->encoding);
        return;
      }
    }
    decoder_->decode(*msg, this);
  }

private:
  // ------------- member variables -------------
  esvo2_Mapping *mapping_{nullptr};
  EventQueue *event_queue_{nullptr};
  uint16_t width_{0};
  uint16_t height_{0};
  bool is_first_in_message_{true};
  event_camera_codecs::Decoder<esvo2_Mapping::EventPacket, EventHandler>
      *decoder_{nullptr};
};

} // namespace esvo2_core

#endif // ESVO2_CORE_MAPPING_H
