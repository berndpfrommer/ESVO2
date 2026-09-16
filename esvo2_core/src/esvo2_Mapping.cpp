#include <esvo2_core/esvo2_Mapping.h>
#include <esvo2_core/factor/pose_local_parameterization.h>
#include <esvo2_core/factor/utility.h>
#include <esvo2_core/tools/params_helper.h>
#include <minkindr_conversions/kindr_tf.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <thread>
#include <utility>

// #define ESVO2_CORE_MAPPING_DEBUG
// #define ESVO2_CORE_MAPPING_LOG

namespace {
geometry_msgs::msg::TransformStamped
poseToTransformStamped(const geometry_msgs::msg::PoseStamped &pose_msg,
                       const std::string &child_frame_id) {
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.frame_id = pose_msg.header.frame_id;
  transform_msg.header.stamp = pose_msg.header.stamp;
  transform_msg.child_frame_id = child_frame_id;
  transform_msg.transform.translation.x = pose_msg.pose.position.x;
  transform_msg.transform.translation.y = pose_msg.pose.position.y;
  transform_msg.transform.translation.z = pose_msg.pose.position.z;
  transform_msg.transform.rotation.x = pose_msg.pose.orientation.x;
  transform_msg.transform.rotation.y = pose_msg.pose.orientation.y;
  transform_msg.transform.rotation.z = pose_msg.pose.orientation.z;
  transform_msg.transform.rotation.w = pose_msg.pose.orientation.w;
  return transform_msg;
}

tf2::TimePoint toTf2TimePoint(esvo2_core::timestamp_t stamp) {
  return tf2::TimePoint(std::chrono::nanoseconds(stamp));
}

esvo2_core::Transformation transformStampedToKindr(
    const geometry_msgs::msg::TransformStamped &transform_msg) {
  const auto &translation = transform_msg.transform.translation;
  const auto &rotation = transform_msg.transform.rotation;
  return esvo2_core::Transformation(
      Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z),
      Eigen::Vector3d(translation.x, translation.y, translation.z));
}

void get_start_and_end_timestamps(bool fromAA, esvo2_core::timestamp_t t,
                                  esvo2_core::timestamp_t *t_begin,
                                  esvo2_core::timestamp_t *t_end,
                                  esvo2_core::timestamp_t hst) {
  if (fromAA) {
    *t_end = t + 5000000;
    *t_begin = t - 5000000;
  } else {
    *t_end = t;
    *t_begin = t - 10 * hst;
  }
}

} // namespace

namespace esvo2_core {
esvo2_Mapping::esvo2_Mapping(const rclcpp::NodeOptions &options)
    : Node("esvo2_mapping", options),
      TS_left_sub_(this, "time_surface_left",
                   rclcpp::QoS(10).get_rmw_qos_profile()),
      TS_right_sub_(this, "time_surface_right",
                    rclcpp::QoS(10).get_rmw_qos_profile()),
      AA_map_sub_(this, "AA_map", rclcpp::QoS(10).get_rmw_qos_profile()),
      TS_negative_sub_(this, "time_surface_negative",
                       rclcpp::QoS(10).get_rmw_qos_profile()),
      TS_dx_sub_(this, "time_surface_negative_dx",
                 rclcpp::QoS(10).get_rmw_qos_profile()),
      TS_dy_sub_(this, "time_surface_negative_dy",
                 rclcpp::QoS(10).get_rmw_qos_profile()),
      TS_sync_(ApproxSyncPolicy(10), TS_left_sub_, TS_dx_sub_),
      TS_AA_sync_(ApproxSyncPolicy2(10), TS_left_sub_, TS_right_sub_,
                  AA_map_sub_, TS_negative_sub_, TS_dx_sub_, TS_dy_sub_),
      calibInfoDir_(tools::param(this, "calibInfoDir", std::string(""))),
      camSysPtr_(new CameraSystem(calibInfoDir_, false)),
      dpConfigPtr_(new DepthProblemConfig(
          tools::param(this, "patch_size_X", 5),
          tools::param(this, "patch_size_Y", 5),
          tools::param(this, "LSnorm_ln", std::string("Tdist")),
          tools::param(this, "Tdist_nu", 0.0),
          tools::param(this, "Tdist_scale", 0.0),
          tools::param(this, "ITERATION_OPTIMIZATION", 1),
          tools::param(this, "RegularizationRadius", 5),
          tools::param(this, "RegularizationMinNeighbours", 8),
          tools::param(this, "RegularizationMinCloseNeighbours", 8))),
      dpSolver_(camSysPtr_, dpConfigPtr_, NUMERICAL, NUM_THREAD_MAPPING, true),
      dFusor_(camSysPtr_, dpConfigPtr_), dRegularizor_(dpConfigPtr_),
      dpConfigPtr_ln_(new DepthProblemConfig(
          tools::param(this, "patch_size_X", 25),
          tools::param(this, "patch_size_Y", 25),
          tools::param(this, "LSnorm_ln", std::string("Tdist")),
          tools::param(this, "Tdist_nu_ln", 0.0),
          tools::param(this, "Tdist_scale_ln", 0.0),
          tools::param(this, "ITERATION_OPTIMIZATION_LN", 1),
          tools::param(this, "RegularizationRadius", 5),
          tools::param(this, "RegularizationMinNeighbours", 8),
          tools::param(this, "RegularizationMinCloseNeighbours", 8))),
      dpSolver_ln_(camSysPtr_, dpConfigPtr_ln_, NUMERICAL, NUM_THREAD_MAPPING,
                   false),
      dFusor_ln_(camSysPtr_, dpConfigPtr_ln_),
      dRegularizor_ln_(dpConfigPtr_ln_),
      ebm_(camSysPtr_, NUM_THREAD_MAPPING,
           tools::param(this, "SmoothTimeSurface", false)),
      pc_near_(new PointCloud()), pc_global_(new PointCloud()),
      depthFramePtr_(new DepthFrame(camSysPtr_->cam_left_ptr_->height_,
                                    camSysPtr_->cam_left_ptr_->width_)),
      BackendOpt_(camSysPtr_) {
  // frame id
  dvs_frame_id_ = tools::param(this, "dvs_frame_id", std::string("dvs"));
  world_frame_id_ = tools::param(this, "world_frame_id", std::string("world"));
  pc_near_->header.frame_id = world_frame_id_;
  pc_global_->header.frame_id = world_frame_id_;
  pc_color_ = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGBL>());
  pc_color_->header.frame_id = world_frame_id_;
  pc_filtered_ = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGBL>());
  pc_filtered_->header.frame_id = world_frame_id_;

  /**** mapping parameters ***/
  // range and visualization threshold
  invDepth_min_range_ = tools::param(this, "invDepth_min_range", 0.16);
  invDepth_max_range_ = tools::param(this, "invDepth_max_range", 2.0);
  patch_area_ = tools::param(this, "patch_size_X", 25) *
                tools::param(this, "patch_size_Y", 25);
  residual_vis_threshold_ = tools::param(this, "residual_vis_threshold", 15);
  residual_vis_threshold_ln_ =
      tools::param(this, "residual_vis_threshold_ln", 15);
  cost_vis_threshold_ = pow(residual_vis_threshold_, 2) * patch_area_;
  cost_vis_threshold_ln_ = pow(residual_vis_threshold_ln_, 2) * patch_area_;
  stdVar_vis_threshold_ = tools::param(this, "stdVar_vis_threshold", 0.005);
  stdVar_vis_threshold_ln_ = tools::param(this, "stdVar_vis_threshold_ln", 1);
  age_max_range_ = tools::param(this, "age_max_range", 5);
  age_vis_threshold_ = tools::param(this, "age_vis_threshold", 0);
  fusion_radius_ = tools::param(this, "fusion_radius", 0);
  maxNumFusionFrames_ = tools::param(this, "maxNumFusionFrames", 10);
  maxNumFusionFrames_ln_ = tools::param(this, "maxNumFusionFrames_ln", 10);
  FusionStrategy_ =
      tools::param(this, "FUSION_STRATEGY", std::string("CONST_FRAMES"));
  maxNumFusionPoints_ = tools::param(this, "maxNumFusionPoints", 2000);
  INIT_SGM_DP_NUM_Threshold_ =
      tools::param(this, "INIT_SGM_DP_NUM_THRESHOLD", 500);

  // options
  bDenoising_ = tools::param(this, "Denoising", false);
  bRegularization_ = tools::param(this, "Regularization", false);
  resetButton_ = tools::param(this, "ResetButton", false);
  blarge_scale_ = tools::param(this, "large_scale", true);
  bpoints_from_AA_ = tools::param(this, "select_points_from_AA", true);
  eta_for_select_points_ = tools::param(this, "eta_for_select_points", 0.1);

  // visualization parameters
  bVisualizeGlobalPC_ = tools::param(this, "bVisualizeGlobalPC", false);
  visualizeGPC_interval_ = tools::param(this, "visualizeGPC_interval", 3);
  visualize_range_ = tools::param(this, "visualize_range", 2.5);
  numAddedPC_threshold_ = tools::param(this, "NumGPC_added_per_refresh", 1000);

  // module parameters
  PROCESS_EVENT_NUM_ = tools::param(this, "PROCESS_EVENT_NUM", 500);
  PROCESS_EVENT_NUM_AA_ = tools::param(this, "PROCESS_EVENT_NUM_AA", 500);
  TS_HISTORY_LENGTH_ = tools::param(this, "TS_HISTORY_LENGTH", 100);
  mapping_rate_hz_ = tools::param(this, "mapping_rate_hz", 20);

  // Event Block Matching (BM) parameters
  BM_half_slice_thickness_ =
      tools::param(this, "BM_half_slice_thickness", 0.001);
  BM_patch_size_X_ = tools::param(this, "patch_size_X", 25);
  BM_patch_size_Y_ = tools::param(this, "patch_size_Y", 25);
  BM_patch_size_X_2_ = tools::param(this, "patch_size_X_2", 25);
  BM_patch_size_Y_2_ = tools::param(this, "patch_size_Y_2", 25);
  x_patches_ = tools::param(this, "x_patches", 8);
  y_patches_ = tools::param(this, "y_patches", 6);
  BM_min_disparity_ = tools::param(this, "BM_min_disparity", 3);
  BM_max_disparity_ = tools::param(this, "BM_max_disparity", 40);
  BM_step_ = tools::param(this, "BM_step", 1);
  BM_ZNCC_Threshold_ = tools::param(this, "BM_ZNCC_Threshold", 0.1);
  BM_bUpDownConfiguration_ =
      tools::param(this, "BM_bUpDownConfiguration", false);
  bUSE_IMU_ = tools::param(this, "USE_IMU", true);

  // distance from last frame
  distance_from_last_frame_ =
      tools::param(this, "distance_from_last_frame", 0.04);

  // SGM parameters (Used by Initialization)
  num_disparities_ = BM_max_disparity_;
  block_size_ = 11;
  P1_ = 8 * 1 * block_size_ * block_size_;
  P2_ = 32 * 1 * block_size_ * block_size_;
  uniqueness_ratio_ = 11;
  sgbm_ = cv::StereoSGBM::create(0, num_disparities_, block_size_, P1_, P2_, -1,
                                 0, uniqueness_ratio_);

  // calcualte the min,max disparity of static block matching
  double f = (camSysPtr_->cam_left_ptr_->P_(0, 0) +
              camSysPtr_->cam_left_ptr_->P_(1, 1)) /
             2;
  double b = camSysPtr_->baseline_;
  size_t minDisparity =
      max(size_t(std::floor(f * b * invDepth_min_range_)), (size_t)0);
  size_t maxDisparity = size_t(std::ceil(f * b * invDepth_max_range_));
  minDisparity = max(minDisparity, BM_min_disparity_);
  maxDisparity = min(maxDisparity, BM_max_disparity_);

  // Backend parameters
  initFirstPoseFlag = false;
  prev_time_ = 0;
  first_imu = false;

  // initialize Event Batch Matcher
  ebm_.resetParameters(BM_patch_size_X_, BM_patch_size_Y_, minDisparity,
                       maxDisparity, BM_step_, BM_ZNCC_Threshold_,
                       BM_bUpDownConfiguration_, BM_patch_size_X_2_,
                       BM_patch_size_Y_2_);
  BM_min_disparity_ = minDisparity;
  BM_max_disparity_ = maxDisparity;
  // system status
  ESVO2_System_Status_ = "INITIALIZATION";
  this->set_parameter(
      rclcpp::Parameter("/ESVO2_SYSTEM_STATUS", ESVO2_System_Status_));

  // callback functions
  stamped_pose_sub_ = this->create_subscription<PoseStamped>(
      "stamped_pose", rclcpp::QoS(1),
      std::bind(&esvo2_Mapping::stampedPoseCallback, this,
                std::placeholders::_1));
  TS_AA_sync_.registerCallback(std::bind(
      &esvo2_Mapping::timeSurfaceCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
      std::placeholders::_5, std::placeholders::_6));

  // point sampling
  if (bpoints_from_AA_) {
    AA_frequency_sub_ = this->create_subscription<Image>(
        "AA_left", rclcpp::QoS(1),
        std::bind(&esvo2_Mapping::AACallback, this, std::placeholders::_1));
  } else {
    events_left_sub_ = this->create_subscription<EventPacket>(
        "events_left", rclcpp::QoS(100),
        std::bind(&esvo2_Mapping::eventsCallback, this, std::placeholders::_1));
  }

  // IMU
  if (bUSE_IMU_)
    imu_sub_ = this->create_subscription<ImuMsg>(
        "imu/data", rclcpp::QoS(1),
        std::bind(&esvo2_Mapping::refImuCallback, this, std::placeholders::_1));

  // TF
  tf_ = std::make_shared<tf2::BufferCore>(tf2::durationFromSec(100.0));

  // result publishers
  invDepthMap_pub_ = image_transport::create_publisher(
      this, "Inverse_Depth_Map2", rmw_qos_profile_default);
  V_ba_bg_pub_ = this->create_publisher<events_repackaging_msgs::msg::Vbabg>(
      "/esvo2_mapping/V_ba_bg", 1);
  pc_pub_ = this->create_publisher<PointCloud2>(
      "/esvo2_mapping/pointcloud_local2", 1);
  pc_filtered_pub_ = this->create_publisher<PointCloud2>(
      "/esvo2_mapping/pointcloud_filtered2", 1);
  if (bVisualizeGlobalPC_) {
    gpc_pub_ = this->create_publisher<PointCloud2>(
        "/esvo2_mapping/pointcloud_global2", 1);
    pc_global_->reserve(5000000);
    t_last_pub_pc_ = 0.0;
  }
  event_handler_ = std::make_shared<EventHandler>(
      this, &events_left_, camSysPtr_->cam_left_ptr_->width_,
      camSysPtr_->cam_left_ptr_->height_);

  // multi-thread management
  mapping_thread_future_ = mapping_thread_promise_.get_future();
  reset_future_ = reset_promise_.get_future();

  // stereo mapping detached thread
  std::thread MappingThread(&esvo2_Mapping::MappingLoop, this,
                            std::move(mapping_thread_promise_),
                            std::move(reset_future_));
  MappingThread.detach();
}

esvo2_Mapping::~esvo2_Mapping() {
  pc_pub_.reset();
  pc_filtered_pub_.reset();
  invDepthMap_pub_.shutdown();
  V_ba_bg_pub_.reset();
}

void esvo2_Mapping::MappingLoop(std::promise<void> prom_mapping,
                                std::future<void> future_reset) {
  rclcpp::Rate r(mapping_rate_hz_);
  while (rclcpp::ok()) {
    // check system status
    this->get_parameter("/ESVO2_SYSTEM_STATUS", ESVO2_System_Status_);
    if (ESVO2_System_Status_ == "TERMINATE") {
      LOG(INFO) << "The Mapping node is terminated manually...";
      break;
    }

    // To assure the esvo2_time_surface node has been working
    if (TS_history_.size() >= 10) {
      TicToc total_mapping;
      while (true) {
        if (data_mutex_.try_lock()) {
          dataTransferring();
          data_mutex_.unlock();
          break;
        } else {
          if (future_reset.wait_for(std::chrono::nanoseconds(1)) ==
              std::future_status::ready) {
            prom_mapping.set_value();
            return;
          }
        }
      }

      // To check if the most current TS observation has been loaded by
      // dataTransferring()
      if (TS_obs_ptr_->second.isEmpty()) {
        r.sleep();
        continue;
      }

      // Do initialization (State Machine)
      if (ESVO2_System_Status_ == "INITIALIZATION" ||
          ESVO2_System_Status_ == "RESET") {
        if (InitializationAtTime(TS_obs_ptr_->first)) {
          LOG(INFO)
              << "Initialization is successfully done!"; //(" <<
                                                         // INITIALIZATION_COUNTER_
                                                         //<< ").";
        } else
          LOG(INFO) << "Initialization fails once.";
      }
      double Data_transfer = total_mapping.toc();

      // Do mapping
      if (ESVO2_System_Status_ == "WORKING")
        MappingAtTime(TS_obs_ptr_->first);

      BackendOpt_.slideWindow();
    } else {
      if (future_reset.wait_for(std::chrono::nanoseconds(1)) ==
          std::future_status::ready) {
        prom_mapping.set_value();
        return;
      }
    }
    r.sleep();
  }
}

void esvo2_Mapping::MappingAtTime(const timestamp_t t) {
  TicToc tt_mapping;
  TicToc mapping_cost; // record the time cost of each step
  double t_overall_count = 0;
  /************************************************/
  /************ set the new DepthFrame ************/
  /************************************************/
  DepthFrame::Ptr depthFramePtr_new = std::make_shared<DepthFrame>(
      camSysPtr_->cam_left_ptr_->height_, camSysPtr_->cam_left_ptr_->width_);
  depthFramePtr_new->setId(TS_obs_ptr_->second.id_);
  depthFramePtr_new->setTransformation(TS_obs_ptr_->second.tr_);
  depthFramePtr_ = depthFramePtr_new;
  std::vector<EventMatchPair>
      vEMP; // the container that stores the result of BM.
  /****************************************************/
  /*************** Block Matching (BM) ****************/
  /****************************************************/
  double t_BM = 0.0;
  double t_BM_denoising = 0.0;
  cv::Mat denoising_mask;

  vDenoisedEventsPtr_left_.clear();
  vDenoisedEventsPtr_left_.reserve(PROCESS_EVENT_NUM_);
  vDenoisedEventsPtr_left_.insert(
      vDenoisedEventsPtr_left_.end(), vCloseEventsPtr_left_.begin(),
      vCloseEventsPtr_left_.begin() +
          min(vCloseEventsPtr_left_.size(), PROCESS_EVENT_NUM_));

  totalNumCount_ = vDenoisedEventsPtr_left_.size();
  t_BM_denoising = tt_mapping.toc();
  t_overall_count += t_BM_denoising;
  tt_mapping.tic();
  double t_BM_select;
  // divide the events into two parts according to gradient.
  selectPoint();
  t_BM_select = tt_mapping.toc();
  t_overall_count += t_BM_select;

  // Denoising operations, only for static stereo matching
  if (bDenoising_) {
    tt_mapping.tic();
    // Draw one mask image for denoising.
    createDenoisingMask(vALLEventsPtr_left_, denoising_mask,
                        camSysPtr_->cam_left_ptr_->height_,
                        camSysPtr_->cam_left_ptr_->width_);
    vDenoisedEventsPtr_left_dx2_.clear();
    // Extract denoised events (appear on edges likely).
    extractDenoisedEvents(vDenoisedEventsPtr_left_dx_,
                          vDenoisedEventsPtr_left_dx2_, denoising_mask,
                          PROCESS_EVENT_NUM_);
    t_BM_denoising += tt_mapping.toc();
  } else {
    vDenoisedEventsPtr_left_dx2_.clear();
    vDenoisedEventsPtr_left_dx2_.insert(
        vDenoisedEventsPtr_left_dx2_.end(), vDenoisedEventsPtr_left_dx_.begin(),
        vDenoisedEventsPtr_left_dx_.begin() +
            min(vDenoisedEventsPtr_left_dx_.size(), PROCESS_EVENT_NUM_));
  }
  t_overall_count += t_BM_denoising;
  tt_mapping.tic();

  // static stereo matching
  ebm_.createMatchProblem(TS_obs_ptr_, &st_map_, &vDenoisedEventsPtr_left_dx2_);
  ebm_.match_all_HyperThread(vEMP);

  std::vector<EventMatchPair> vEMP_last_pre, vEMP_last, vEMP_last_fail;
  vEMP_last_pre.reserve(vDenoisedEventsPtr_left_dy_.size());
  vEMP_last.reserve(vDenoisedEventsPtr_left_dy_.size());
  vEMP_last_fail.reserve(vDenoisedEventsPtr_left_dy_.size());

  // determine whether the last frame is available
  if (!(TS_obs_ptr_->second.TS_last_.rows() == 0 ||
        TS_obs_ptr_->second.TS_last_.cols() == 0 ||
        (TS_obs_ptr_->second.tr_last_.getPosition() -
         TS_obs_ptr_->second.tr_.getPosition())
                .norm() < distance_from_last_frame_)) {
    Eigen::Matrix4d T_last_now =
        TS_obs_ptr_->second.tr_last_.getTransformationMatrix().inverse() *
        TS_obs_ptr_->second.tr_.getTransformationMatrix();

    // get the direction of epipolar line
    getReprojection(vEMP_last_pre, T_last_now, vDenoisedEventsPtr_left_dy_);
    cv::cv2eigen(TS_obs_ptr_->second.cvImagePtr_AA_map_->image.clone(),
                 TS_obs_ptr_->second.AA_map_);
    cv::cv2eigen(TS_obs_ptr_->second.cvImagePtr_last_->image.clone(),
                 TS_obs_ptr_->second.TS_last_);

    // temporal stereo matching
    ebm_.createMatchProblemTwoFrames(
        TS_obs_ptr_, &st_map_, &vDenoisedEventsPtr_left_dy_, &vEMP_last_pre);
    ebm_.match_all_HyperThreadTwoFrames(vEMP_last, vEMP_last_fail);
  }

  t_BM = tt_mapping.toc();
  t_overall_count += t_BM_denoising;
  t_overall_count += t_BM;

  /**************************************************************/
  /*************  Fusion ***************/
  /**************************************************************/
  double t_optimization = 0;
  double t_solve, t_fusion, t_regularization;
  t_solve = t_fusion = t_regularization = 0;
  size_t numFusionCount = 0,
         numFusionCount_ln =
             0; // To count the total number of fusion (in terms of fusion
                // between two estimates, i.e. a priori and a propagated one).
  tt_mapping.tic();

  // just compute the variance of the residual
  std::vector<DepthPoint> vdp, vdp_ln;
  vdp.reserve(vEMP.size() + vEMP_last.size());
  dpSolver_.solve(&vEMP, TS_obs_ptr_, vdp);
  vdp_ln.reserve(vEMP_last.size());
  dpSolver_ln_.solve(&vEMP_last, TS_obs_ptr_, vdp_ln);

  dpSolver_.pointCulling(vdp, stdVar_vis_threshold_, cost_vis_threshold_,
                         invDepth_min_range_, invDepth_max_range_);
  if (blarge_scale_)
    dpSolver_ln_.pointCulling(vdp_ln, stdVar_vis_threshold_ln_,
                              cost_vis_threshold_ln_, invDepth_min_range_,
                              invDepth_max_range_);

  t_solve = tt_mapping.toc();
  tt_mapping.tic();

  if (FusionStrategy_ ==
      "CONST_POINTS") // Fusion (strategy 1: const number of point)
  {
    size_t numFusionPoints = 0;
    DepthPointFrame vdpf(t, vdp);
    dqvDepthPoints_.push_back(vdpf);
    for (size_t n = 0; n < dqvDepthPoints_.size(); n++)
      numFusionPoints += dqvDepthPoints_[n].size();
    while (numFusionPoints > 1.5 * maxNumFusionPoints_) {
      dqvDepthPoints_.pop_front();
      numFusionPoints = 0;
      for (size_t n = 0; n < dqvDepthPoints_.size(); n++)
        numFusionPoints += dqvDepthPoints_[n].size();
    }
  } else if (FusionStrategy_ ==
             "CONST_FRAMES") // (strategy 2: const number of frames)
  {
    DepthPointFrame vdpf(t, vdp), vdpf_ln(t, vdp_ln);
    dqvDepthPoints_.push_back(vdpf);
    dqvDepthPoints_ln_.push_back(vdpf_ln);
    while (dqvDepthPoints_.size() > maxNumFusionFrames_)
      dqvDepthPoints_.pop_front();
    while (dqvDepthPoints_ln_.size() > maxNumFusionFrames_ln_)
      dqvDepthPoints_ln_.pop_front();
  } else
    LOG(INFO) << "Invalid FusionStrategy is assigned.";

  // apply fusion and count the total number of fusion.
  numFusionCount = 0;
  int total = 0;
  for (auto it = dqvDepthPoints_.rbegin(); it != dqvDepthPoints_.rend(); it++) {
    total += it->size();
    numFusionCount +=
        dFusor_.update(it->DepthPoints_, depthFramePtr_, fusion_radius_);
    if (blarge_scale_)
      for (int i = 0; i < 3; i++) {
        if (it != dqvDepthPoints_.rend() - 1)
          it++;
      }
  }
  TotalNumFusion_ += numFusionCount + numFusionCount_ln;
  if (dqvDepthPoints_.size() >= maxNumFusionFrames_)
    depthFramePtr_->dMap_->clean(pow(stdVar_vis_threshold_, 2),
                                 age_vis_threshold_, invDepth_max_range_,
                                 invDepth_min_range_);

  double data_time = tt_mapping.toc();

  // regularization
  if (bRegularization_) {
    tt_mapping.tic();
    dRegularizor_.apply(depthFramePtr_->dMap_);
    t_regularization = tt_mapping.toc();
  }
  tt_mapping.tic();
  for (auto it = dqvDepthPoints_ln_.rbegin(); it != dqvDepthPoints_ln_.rend();
       it++) {
    total += it->size();
    numFusionCount_ln +=
        dFusor_ln_.update(it->DepthPoints_, depthFramePtr_, fusion_radius_);
  }

  // count time
  t_fusion = tt_mapping.toc() + data_time;
  t_optimization = t_solve + t_fusion + t_regularization;
  t_overall_count += t_optimization;

  // publish results
  TicToc t_optimize;
  if (dqvDepthPoints_.size() >= WINDOW_SIZE + 1 && bUSE_IMU_ == true) {
    BackendOpt_.setProblem(&dqvDepthPoints_, &TS_history_, V_ba_bg_pub_.get(),
                           bUSE_IMU_);
    BackendOpt_.sloveProblem();
  }
  double time_optimize = t_optimize.toc();
  t_overall_count += time_optimize;

  std::thread tPublishMappingResult(&esvo2_Mapping::publishMappingResults, this,
                                    depthFramePtr_->dMap_,
                                    depthFramePtr_->T_world_frame_, t);
  tPublishMappingResult.detach();
#ifdef ESVO2_CORE_MAPPING_LOG
  LOG(INFO) << "\n";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "--------------Computation Cost (Mapping)---------------------";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "Denoising: " << t_BM_denoising << " ms, ("
            << t_BM_denoising / t_overall_count * 100 << "%).";
  LOG(INFO) << "Point Selection (BM): " << t_BM_select << " ms, ("
            << t_BM_select / t_overall_count * 100 << "%).";
  LOG(INFO) << "Block Matching (BM): " << t_BM << " ms, ("
            << t_BM / t_overall_count * 100 << "%).";
  LOG(INFO) << "BM success ratio: " << vEMP.size() + vEMP_last.size() << "/"
            << totalNumCount_ << "(Successes/Total).";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "Update: " << t_optimization << " ms, ("
            << t_optimization / t_overall_count * 100 << "%).";
  LOG(INFO) << "-- compute variance: " << t_solve << " ms, ("
            << t_solve / t_overall_count * 100 << "%).";
  LOG(INFO) << "-- fusion (" << numFusionCount << ", " << TotalNumFusion_
            << "): " << t_fusion << " ms, (" << t_fusion / t_overall_count * 100
            << "%).";
  LOG(INFO) << "-- regularization: " << t_regularization << " ms, ("
            << t_regularization / t_overall_count * 100 << "%).";
  LOG(INFO) << "-- time_optimize: " << time_optimize << " ms, ("
            << time_optimize / t_overall_count * 100 << "%).";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "Total Computation (" << depthFramePtr_->dMap_->size()
            << "): " << t_overall_count << " ms. ----" << mapping_cost.toc()
            << " ms";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "------------------------------END---------------------------";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "\n";
#endif
}

bool esvo2_Mapping::InitializationAtTime(const timestamp_t t) {
  // create a new depth frame
  DepthFrame::Ptr depthFramePtr_new = std::make_shared<DepthFrame>(
      camSysPtr_->cam_left_ptr_->height_, camSysPtr_->cam_left_ptr_->width_);
  depthFramePtr_new->setId(TS_obs_ptr_->second.id_);
  depthFramePtr_new->setTransformation(TS_obs_ptr_->second.tr_);
  depthFramePtr_ = depthFramePtr_new;

  // call SGM on the current Time Surface observation pair.
  cv::Mat dispMap, dispMap8;
  sgbm_->compute(TS_obs_ptr_->second.cvImagePtr_left_->image,
                 TS_obs_ptr_->second.cvImagePtr_right_->image, dispMap);
  dispMap.convertTo(dispMap8, CV_8U, 255 / (num_disparities_ * 16.));

  // get the event map (binary mask)
  cv::Mat edgeMap;
  std::vector<std::pair<size_t, size_t>> vEdgeletCoordinates;
  createEdgeMask(vEventsPtr_left_SGM_, camSysPtr_->cam_left_ptr_, edgeMap,
                 vEdgeletCoordinates, true, 0);

  // Apply logical "AND" operation and transfer "disparity" to "invDepth".
  std::vector<DepthPoint> vdp_sgm;
  vdp_sgm.reserve(vEdgeletCoordinates.size());
  double var_SGM = pow(0.001, 2);
  for (size_t i = 0; i < vEdgeletCoordinates.size(); i++) {
    size_t x = vEdgeletCoordinates[i].first;
    size_t y = vEdgeletCoordinates[i].second;

    double disp = dispMap.at<short>(y, x) / 16.0;
    if (disp < 0)
      continue;
    DepthPoint dp(x, y);
    Eigen::Vector2d p_img(x * 1.0, y * 1.0);
    dp.update_x(p_img);
    double invDepth =
        disp / (camSysPtr_->cam_left_ptr_->P_(0, 0) * camSysPtr_->baseline_);
    if (invDepth < invDepth_min_range_ || invDepth > invDepth_max_range_)
      continue;
    Eigen::Vector3d p_cam;
    camSysPtr_->cam_left_ptr_->cam2World(p_img, invDepth, p_cam);
    dp.update_p_cam(p_cam);
    dp.update(invDepth,
              var_SGM); // assume the statics of the SGM's results are Guassian.
    dp.residual() = 0.0;
    dp.age() = age_vis_threshold_;
    Eigen::Matrix<double, 4, 4> T_world_cam =
        TS_obs_ptr_->second.tr_.getTransformationMatrix();
    dp.updatePose(T_world_cam);
    vdp_sgm.push_back(dp);
  }
  LOG(INFO) << vEventsPtr_left_SGM_.size()
            << "********** Initialization (SGM) returns " << vdp_sgm.size()
            << " points.";
  if (vdp_sgm.size() < INIT_SGM_DP_NUM_Threshold_)
    return false;
  // push the "masked" SGM results to the depthFrame
  DepthPointFrame vdpf_sgm(t, vdp_sgm);
  dqvDepthPoints_.push_back(vdpf_sgm);
  dFusor_.naive_propagation(vdp_sgm, depthFramePtr_);
  // publish the invDepth map
  std::thread tPublishMappingResult(&esvo2_Mapping::publishMappingResults, this,
                                    depthFramePtr_->dMap_,
                                    depthFramePtr_->T_world_frame_, t);
  tPublishMappingResult.detach();
  return true;
}

bool esvo2_Mapping::dataTransferring() {
  TS_obs_ptr_ = NULL; // clean the TS obs.
  constStampedTimeSurfaceObs emptyObs;
  TS_obs_ptr_ = reinterpret_cast<constStampedTimeSurfaceObs *>(&emptyObs);

  // To assure the esvo2_time_surface node has been working.
  if (TS_history_.size() <= 10)
    return false;
  totalNumCount_ = 0;

  // load current Time-Surface Observation
  auto it_end = TS_history_.rbegin();
  it_end++; // in case that the tf is behind the most current TS.
  auto it_begin = TS_history_.begin();
  while (TS_obs_ptr_->second.isEmpty()) {
    Transformation tr, tr_last;
    if (ESVO2_System_Status_ == "INITIALIZATION") {
      tr.setIdentity();
      it_end->second.setTransformation(tr);
      it_end->second.setOriTransformation(tr);
      TS_obs_ptr_ = &(*it_end);
    }
    if (ESVO2_System_Status_ == "WORKING") {
      if (getPoseAt(it_end->first, tr, dvs_frame_id_)) {
        it_end->second.setTransformation(tr);
        it_end->second.setOriTransformation(tr);
        TS_obs_ptr_ = &(*it_end);
        while (it_end->first != it_begin->first) {
          if (getPoseAt(it_end->first, tr_last, dvs_frame_id_)) {
            if ((tr_last.getPosition() - tr.getPosition()).norm() >
                distance_from_last_frame_) {
              it_end->second.setTransformation(tr_last);
              if (!it_end->second.isEmpty()) {
                TS_obs_ptr_->second.tr_last_ = it_end->second.tr_;
                TS_obs_ptr_->second.TS_last_ = it_end->second.AA_map_;
                TS_obs_ptr_->second.TS_last_du = it_end->second.dTS_du_left_;
                TS_obs_ptr_->second.TS_last_dv = it_end->second.dTS_dv_left_;
                TS_obs_ptr_->second.cvImagePtr_last_ =
                    it_end->second.cvImagePtr_AA_map_;
              }

              break;
            }
          }
          it_end++;
        }
      } else {
        // check if the tracking node is still working normally
        this->get_parameter("/ESVO2_SYSTEM_STATUS", ESVO2_System_Status_);
        if (ESVO2_System_Status_ != "WORKING")
          return false;
      }
    }
    if (it_end->first == it_begin->first)
      break;
    it_end++;
  }
  if (TS_obs_ptr_->second.isEmpty())
    return false;

  std::vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
  const auto curTime = TS_obs_ptr_->first;
  if (prev_time_ == 0)
    prev_time_ = TS_obs_ptr_->first - 500000000LL;
  mBuf.lock();
  // get the IMU data by time interval
  getIMUInterval(prev_time_, curTime, accVector, gyrVector);
  mBuf.unlock();
  if (!initFirstPoseFlag)
    initFirstIMUPose(accVector);
  for (int i = 0; i < accVector.size(); i++) {
    double dt;
    if (i == 0)
      dt = accVector[i].first - prev_time_;
    else if (i == accVector.size() - 1)
      dt = curTime - accVector[i - 1].first;
    else
      dt = accVector[i].first - accVector[i - 1].first;

    // imu pre-integration
    processIMU(accVector[i].first, dt, accVector[i].second,
               gyrVector[i].second);
  }
  prev_time_ = curTime;

  /****** Load involved events *****/
  // SGM
  if (ESVO2_System_Status_ == "INITIALIZATION") {
    vEventsPtr_left_SGM_.clear();
    timestamp_t t_end, t_begin;
    get_start_and_end_timestamps(bpoints_from_AA_, TS_obs_ptr_->first, &t_begin,
                                 &t_end, BM_half_slice_thickness_);
    auto ev_end_it = tools::EventBuffer_lower_bound(events_left_, t_end);
    auto ev_begin_it = tools::EventBuffer_lower_bound(events_left_, t_begin);
    const size_t MAX_NUM_Event_INVOLVED = 30000;
    vEventsPtr_left_SGM_.reserve(MAX_NUM_Event_INVOLVED);
    while (ev_begin_it != ev_end_it &&
           vEventsPtr_left_SGM_.size() <= PROCESS_EVENT_NUM_) {
      vEventsPtr_left_SGM_.push_back(ev_begin_it._M_cur);
      ev_begin_it++;
    }
  }

  // BM
  if (ESVO2_System_Status_ == "WORKING") {
    // copy all involved events' pointers
    vALLEventsPtr_left_.clear();   // Used to generate denoising mask (only used
                                   // to deal with flicker induced by VICON.)
    vCloseEventsPtr_left_.clear(); // Will be denoised using the mask above.

    // load allEvent
    timestamp_t t_end, t_begin;
    get_start_and_end_timestamps(bpoints_from_AA_, TS_obs_ptr_->first, &t_begin,
                                 &t_end, BM_half_slice_thickness_);
    auto ev_begin_it = tools::EventBuffer_lower_bound(events_left_, t_begin);
    auto ev_end_it = tools::EventBuffer_lower_bound(events_left_, t_end);

    const size_t MAX_NUM_Event_INVOLVED = PROCESS_EVENT_NUM_;
    vALLEventsPtr_left_.reserve(MAX_NUM_Event_INVOLVED);
    vCloseEventsPtr_left_.reserve(MAX_NUM_Event_INVOLVED);
    while (ev_end_it != ev_begin_it &&
           vALLEventsPtr_left_.size() < MAX_NUM_Event_INVOLVED) {
      vALLEventsPtr_left_.push_back(ev_end_it._M_cur);
      vCloseEventsPtr_left_.push_back(ev_end_it._M_cur);
      ev_end_it--;
    }
    totalNumCount_ = vCloseEventsPtr_left_.size();
#ifdef ESVO2_CORE_MAPPING_DEBUG
    LOG(INFO) << "Data Transferring (events_left_): " << events_left_.size();
    LOG(INFO) << "Data Transferring (vALLEventsPtr_left_): "
              << vALLEventsPtr_left_.size();
    LOG(INFO) << "Data Transforming (vCloseEventsPtr_left_): "
              << vCloseEventsPtr_left_.size();
#endif
    if (vCloseEventsPtr_left_.size() < 100) {
      return false;
    }

#ifdef ESVO2_CORE_MAPPING_DEBUG
    LOG(INFO) << "Data Transferring (stampTransformation map): "
              << st_map_.size();
#endif
  }
  return true;
}

void esvo2_Mapping::stampedPoseCallback(const PosePtr &ps_msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // To check inconsistent timestamps and reset.
  static constexpr double max_time_diff_before_reset_s = 0.5;
  const timestamp_t stamp_first_event =
      rclcpp::Time(ps_msg->header.stamp).nanoseconds();
  std::string *err_tf = new std::string();
  delete err_tf;

  if (tf_lastest_common_time_ != 0) {
    const timestamp_t dt = stamp_first_event - tf_lastest_common_time_;
    if (dt < 0 || std::abs<timestamp_t>(dt) >= max_time_diff_before_reset_s) {
      RCLCPP_INFO(
          get_logger(),
          "Inconsistent event timestamps detected <stampedPoseCallback> "
          "(new: %ld, old %ld), resetting.",
          stamp_first_event, tf_lastest_common_time_);
      reset();
    }
  }

  const auto transform_msg = poseToTransformStamped(*ps_msg, dvs_frame_id_);
  tf_->setTransform(transform_msg, "esvo2_mapping");
}

// return the pose of the left event cam at time t.
bool esvo2_Mapping::getPoseAt(const timestamp_t t,
                              esvo2_core::Transformation &Tr, // T_world_virtual
                              const std::string &source_frame) {
  std::string err_msg;
  if (!tf_->canTransform(world_frame_id_, source_frame, toTf2TimePoint(t),
                         &err_msg)) {
#ifdef ESVO2_CORE_MAPPING_LOG
    LOG(WARNING) << t.toNSec() << " : " << err_msg;
#endif
    return false;
  } else {
    const auto transform_msg =
        tf_->lookupTransform(world_frame_id_, source_frame, toTf2TimePoint(t));
    Tr = transformStampedToKindr(transform_msg);
    return true;
  }
}

void esvo2_Mapping::eventsCallback(const EventPacketPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  event_handler_->handleMessage(msg); // updates events_left_;
  clearEventQueue(events_left_);
}

void esvo2_Mapping::clearEventQueue(EventQueue &EQ) {
  static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 3000000;
  if (EQ.size() > MAX_EVENT_QUEUE_LENGTH) {
    size_t NUM_EVENTS_TO_REMOVE = EQ.size() - MAX_EVENT_QUEUE_LENGTH;
    EQ.erase(EQ.begin(), EQ.begin() + NUM_EVENTS_TO_REMOVE);
  }
}

void esvo2_Mapping::timeSurfaceCallback(
    const ImagePtr &time_surface_left, const ImagePtr &time_surface_right,
    const ImagePtr &AA_map, const ImagePtr &time_surface_negative,
    const ImagePtr &time_surface_negative_dx,
    const ImagePtr &time_surface_negative_dy) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // check time-stamp inconsistency
  if (!TS_history_.empty()) {
    constexpr timestamp_t max_time_diff_before_reset_s =
        1000000000LL; // 1 second in nanoseconds
    const timestamp_t stamp_last_image = TS_history_.rbegin()->first;
    const timestamp_t dt =
        rclcpp::Time(time_surface_left->header.stamp).nanoseconds() -
        stamp_last_image;
    if (dt < 0 || std::abs<timestamp_t>(dt) >= max_time_diff_before_reset_s) {
      RCLCPP_INFO(get_logger(),
                  "Inconsistent frame timestamp detected <timeSurfaceCallback> "
                  "(new: %ld, old %ld), resetting.",
                  rclcpp::Time(time_surface_left->header.stamp).nanoseconds(),
                  stamp_last_image);
      reset();
    }
  }
  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right, cv_ptr_AA_map_left,
      cv_ptr_negative, cv_ptr_negative_dx, cv_ptr_negative_dy;
  try {
    cv_ptr_left = cv_bridge::toCvCopy(time_surface_left,
                                      sensor_msgs::image_encodings::MONO8);
    cv_ptr_right = cv_bridge::toCvCopy(time_surface_right,
                                       sensor_msgs::image_encodings::MONO8);
    cv_ptr_AA_map_left =
        cv_bridge::toCvCopy(AA_map, sensor_msgs::image_encodings::MONO8);
    cv_ptr_negative = cv_bridge::toCvCopy(time_surface_negative,
                                          sensor_msgs::image_encodings::MONO8);
    cv_ptr_negative_dx = cv_bridge::toCvCopy(
        time_surface_negative_dx, sensor_msgs::image_encodings::TYPE_16SC1);
    cv_ptr_negative_dy = cv_bridge::toCvCopy(
        time_surface_negative_dy, sensor_msgs::image_encodings::TYPE_16SC1);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  // push back the new time surface map
  const timestamp_t t_new_TS =
      rclcpp::Time(time_surface_left->header.stamp).nanoseconds();

  // Made the gradient computation optional which is up to the jacobian choice.
  if (dpSolver_.getProblemType() == NUMERICAL ||
      dpSolver_ln_.getProblemType() == NUMERICAL)
    TS_history_.emplace(
        t_new_TS,
        TimeSurfaceObservation(cv_ptr_left, cv_ptr_right, cv_ptr_AA_map_left,
                               cv_ptr_negative, cv_ptr_negative_dx,
                               cv_ptr_negative_dy, TS_id_, false));
  else
    TS_history_.emplace(
        t_new_TS,
        TimeSurfaceObservation(cv_ptr_left, cv_ptr_right, cv_ptr_AA_map_left,
                               cv_ptr_negative, cv_ptr_negative_dx,
                               cv_ptr_negative_dy, TS_id_, true));
  // keep TS_history's size constant
  while (TS_history_.size() > TS_HISTORY_LENGTH_) {
    auto it = TS_history_.begin();
    TS_history_.erase(it);
  }
}

void esvo2_Mapping::AACallback(const ImagePtr &AA_left) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // check timestamp consistency
  if (!TS_history_.empty()) {
    static constexpr double max_time_diff_before_reset_s = 1;
    const auto stamp_last_image = TS_history_.rbegin()->first;
    const timestamp_t dt =
        rclcpp::Time(AA_left->header.stamp).nanoseconds() - stamp_last_image;
    if (std::abs(dt) >= max_time_diff_before_reset_s) {
      RCLCPP_INFO(
          get_logger(),
          "Inconsistent frame timestamp detected <AACallback> (new: %ld, "
          "old %ld), resetting.",
          rclcpp::Time(AA_left->header.stamp).nanoseconds(), stamp_last_image);
      reset();
    }
  }

  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
  try {
    cv_ptr_left =
        cv_bridge::toCvCopy(AA_left, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // select the pixels with high event frequency
  int num_of_resultImg = 0, drift_t = 0;
  double persent_of_point = 1;
  EventQueue EQ_tmp;
  cv::Mat resultImg = cv_ptr_left->image.clone();

  std::vector<std::vector<std::pair<int, cv::Point>>> roi_events(x_patches_ *
                                                                 y_patches_);
  std::vector<int> num_of_roi(x_patches_ * y_patches_, 0);
  cv::Mat AA = cv::Mat::zeros(resultImg.size(), resultImg.type());
  std::vector<int> num_processed(x_patches_ * y_patches_, 0);
  for (int y = 0; y < resultImg.rows; y++) {
    for (int x = 0; x < resultImg.cols; x++) {
      if (resultImg.at<uchar>(y, x) > 0) {
        num_of_resultImg++;
        int index =
            (y / (int)ceil((double)resultImg.rows / (double)y_patches_)) *
                (x_patches_) +
            (x / (int)ceil((double)resultImg.cols / (double)x_patches_));
        num_of_roi[index]++;
        roi_events[index].push_back(
            std::make_pair((int)resultImg.at<uchar>(y, x), cv::Point(x, y)));
      }
    }
  }
  std::vector<double> ratios(x_patches_ * y_patches_, 0);
  cv::Mat events_map =
      cv::Mat::zeros(cv_ptr_left->image.size(), cv_ptr_left->image.type());
  cv::cvtColor(events_map, events_map, cv::COLOR_GRAY2BGR);
  std::random_device rd;
  std::mt19937 generator(rd());
  for (int i = 0; i < (x_patches_ * y_patches_); i++) {
    // shuffle and sort the event points to ensure sampling uniformity as much
    // as possible
    std::shuffle(roi_events[i].begin(), roi_events[i].end(), generator);
    sort(roi_events[i].begin(), roi_events[i].end(),
         [](std::pair<double, cv::Point> a, std::pair<double, cv::Point> b) {
           return (a.first > b.first);
         });
    ratios[i] = (double)num_of_roi[i] / (double)num_of_resultImg * 0.75;
    for (int j = 0; j < std::min((size_t)(PROCESS_EVENT_NUM_AA_ * ratios[i]),
                                 roi_events[i].size() / 2);
         j++) {
      const auto &rev = roi_events[i][j].second;
      const Event ev(rev.x, rev.y,
                     rclcpp::Time(AA_left->header.stamp).nanoseconds() + 1, 0);
      EQ_tmp.push_back(ev);
      AA.at<uchar>(ev.y, ev.x) = 255;
      num_processed[i] = j;
    }
  }

  int empty_num = PROCESS_EVENT_NUM_AA_ - EQ_tmp.size();
  if (empty_num > 0) {
    for (int i = 0; i < (x_patches_ * y_patches_); i++) {
      persent_of_point =
          std::min(((double)empty_num) / x_patches_ * y_patches_, 1.);
      for (int j = num_processed[i];
           j < std::min((size_t)(PROCESS_EVENT_NUM_AA_ * ratios[i]) +
                            empty_num / (x_patches_ * y_patches_),
                        roi_events[i].size() / 2);
           j++) {
        const auto &rev = roi_events[i][j].second;
        const Event ev(rev.x, rev.y,
                       rclcpp::Time(AA_left->header.stamp).nanoseconds() + 1,
                       0);
        EQ_tmp.push_back(ev);
        AA.at<uchar>(ev.y, ev.x) = 255;
        drift_t++;
      }
    }
  }
  for (const auto &e : EQ_tmp)
    events_left_.push_back(e);
  clearEventQueue(events_left_);
}

void esvo2_Mapping::refImuCallback(const ImuPtr &msg) {
  double t = rclcpp::Time(msg->header.stamp).seconds();
  double dx = msg->linear_acceleration.x;
  double dy = msg->linear_acceleration.y;
  double dz = msg->linear_acceleration.z;
  double rx = msg->angular_velocity.x;
  double ry = msg->angular_velocity.y;
  double rz = msg->angular_velocity.z;
  Eigen::Vector3d acc(dx, dy, dz);
  Eigen::Vector3d gyr(rx, ry, rz);
  mBuf.lock();
  accBuf.push(make_pair(t, acc));
  gyrBuf.push(make_pair(t, gyr));
  mBuf.unlock();
  return;
}

void esvo2_Mapping::reset() {
  // mutual-thread communication with MappingThread.
  LOG(INFO) << "Coming into reset()";
  reset_promise_.set_value();
  LOG(INFO) << "(reset) The mapping thread future is waiting for the value.";
  mapping_thread_future_.get();
  LOG(INFO) << "(reset) The mapping thread future receives the value.";

  // clear all maintained data
  events_left_.clear();
  events_right_.clear();
  TS_history_.clear();
  tf_->clear();
  pc_color_->clear();
  pc_filtered_->clear();
  pc_near_->clear();
  pc_global_->clear();
  TS_id_ = 0;
  depthFramePtr_->clear();
  dqvDepthPoints_.clear();

  ebm_.resetParameters(BM_patch_size_X_, BM_patch_size_Y_, BM_min_disparity_,
                       BM_max_disparity_, BM_step_, BM_ZNCC_Threshold_,
                       BM_bUpDownConfiguration_, BM_patch_size_X_2_,
                       BM_patch_size_Y_2_);

  for (int i = 0; i < 2; i++)
    LOG(INFO) << "****************************************************";
  LOG(INFO) << "****************** RESET THE SYSTEM *********************";
  for (int i = 0; i < 2; i++)
    LOG(INFO) << "****************************************************\n\n";

  // restart the mapping thread
  reset_promise_ = std::promise<void>();
  mapping_thread_promise_ = std::promise<void>();
  reset_future_ = reset_promise_.get_future();
  mapping_thread_future_ = mapping_thread_promise_.get_future();
  ESVO2_System_Status_ = "INITIALIZATION";
  this->set_parameter(
      rclcpp::Parameter("/ESVO2_SYSTEM_STATUS", ESVO2_System_Status_));

  std::thread MappingThread(&esvo2_Mapping::MappingLoop, this,
                            std::move(mapping_thread_promise_),
                            std::move(reset_future_));
  MappingThread.detach();
}

void esvo2_Mapping::publishMappingResults(DepthMap::Ptr depthMapPtr,
                                          Transformation tr, timestamp_t t) {
  cv::Mat invDepthImage, stdVarImage, ageImage, costImage, eventImage,
      confidenceMap, invDepthImage_rel;

  invDepthImage = TS_obs_ptr_->second.cvImagePtr_left_->image.clone();
  visualizor_.plot_map(depthMapPtr, tools::InvDepthMap, invDepthImage,
                       invDepth_max_range_, invDepth_min_range_,
                       stdVar_vis_threshold_, age_vis_threshold_);
  publishImage(invDepthImage, t, invDepthMap_pub_);

  if (ESVO2_System_Status_ == "INITIALIZATION")
    publishPointCloud(depthMapPtr, tr, t);
  if (ESVO2_System_Status_ == "WORKING") {
    if (FusionStrategy_ == "CONST_FRAMES") {
      if (dqvDepthPoints_.size() == maxNumFusionFrames_)
        publishPointCloud(depthMapPtr, tr, t);
    }
    if (FusionStrategy_ == "CONST_POINTS") {
      size_t numFusionPoints = 0;
      for (size_t n = 0; n < dqvDepthPoints_.size(); n++)
        numFusionPoints += dqvDepthPoints_[n].size();
      if (numFusionPoints > 0.5 * maxNumFusionPoints_)
        publishPointCloud(depthMapPtr, tr, t);
    }
  }
}

void esvo2_Mapping::publishPointCloud(DepthMap::Ptr &depthMapPtr,
                                      Transformation &tr, timestamp_t t) {
  PointCloud2::SharedPtr pc_to_publish(new PointCloud2);
  Eigen::Matrix<double, 4, 4> T_world_result = tr.getTransformationMatrix();

  pc_color_->clear();
  pc_color_->reserve(depthMapPtr->size());
  pc_filtered_->clear();
  pc_filtered_->reserve(depthMapPtr->size());
  pc_near_->clear();
  pc_near_->reserve(depthMapPtr->size());

  double FarthestDistance = 0.0;
  Eigen::Vector3d FarthestPoint;

  for (auto it = depthMapPtr->begin(); it != depthMapPtr->end(); it++) {
    Eigen::Vector3d p_world = T_world_result.block<3, 3>(0, 0) * it->p_cam() +
                              T_world_result.block<3, 1>(0, 3);

    // set color for each point
    int index = floor((1 / it->p_cam().z() - invDepth_min_range_) /
                      (invDepth_max_range_ - invDepth_min_range_) * 255.0f);
    if (index > 255)
      index = 255;
    if (index < 0)
      index = 0;
    pcl::PointXYZRGBL point;
    point.x = p_world(0);
    point.y = p_world(1);
    point.z = p_world(2);
    point.r = 255.0f * Visualization::r[index];
    point.g = 255.0f * Visualization::g[index];
    point.b = 255.0f * Visualization::b[index];

    // set label for each point: 1 for shown, 0 for hidden
    if (it->valid() && it->variance() < pow(stdVar_vis_threshold_, 2) &&
        it->age() >= (int)age_vis_threshold_) {
      point.label = 1;
      pc_filtered_->push_back(point);
      if (it->p_cam().norm() < visualize_range_)
        pc_near_->push_back(pcl::PointXYZ(p_world(0), p_world(1), p_world(2)));
    } else
      point.label = 0;
    pc_color_->push_back(point);
  }

  // publish the local 3D map which is used by the tracker.
  if (!pc_color_->empty()) {
    pcl::toROSMsg(*pc_color_, *pc_to_publish);
    pc_to_publish->header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
    pc_pub_->publish(*pc_to_publish);
  }
  if (!pc_filtered_->empty()) {
    pcl::toROSMsg(*pc_filtered_, *pc_to_publish);
    pc_to_publish->header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
    pc_filtered_pub_->publish(*pc_to_publish);
  }

  // publish global pointcloud
  if (bVisualizeGlobalPC_) {
    if (t - t_last_pub_pc_ > visualizeGPC_interval_) {
      PointCloud::Ptr pc_filtered(new PointCloud());
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud(pc_near_);
      if (blarge_scale_)
        sor.setLeafSize(0.3, 0.3, 0.3); // Used in small scale environment.
      else
        sor.setLeafSize(0.01, 0.01, 0.01); // Used in large scale environment.
      sor.filter(*pc_filtered);

      // copy the most current pc tp pc_global
      size_t pc_length = pc_filtered->size();
      size_t numAddedPC = min(pc_length, numAddedPC_threshold_) - 1;
      pc_global_->insert(pc_global_->end(), pc_filtered->end() - numAddedPC,
                         pc_filtered->end());
      pcl::PCDWriter writer;

      // publish point cloud
      pcl::toROSMsg(*pc_global_, *pc_to_publish);
      pc_to_publish->header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
      gpc_pub_->publish(*pc_to_publish);
      t_last_pub_pc_ = t;
    }
  }
}

void esvo2_Mapping::publishImage(const cv::Mat &image, const timestamp_t t,
                                 image_transport::Publisher &pub,
                                 std::string encoding) {
  if (pub.getNumSubscribers() == 0)
    return;

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
  ImagePtr msg =
      cv_bridge::CvImage(header, encoding.c_str(), image).toImageMsg();
  pub.publish(msg);
}

void esvo2_Mapping::createEdgeMask(
    std::vector<Event *> &vEventsPtr, PerspectiveCamera::Ptr &camPtr,
    cv::Mat &edgeMap,
    std::vector<std::pair<size_t, size_t>> &vEdgeletCoordinates,
    bool bUndistortEvents, size_t radius) {
  size_t col = camPtr->width_;
  size_t row = camPtr->height_;
  int dilate_radius = (int)radius;
  edgeMap = cv::Mat(cv::Size(col, row), CV_8UC1, cv::Scalar(0));
  vEdgeletCoordinates.reserve(col * row);

  auto it_tmp = vEventsPtr.begin();
  while (it_tmp != vEventsPtr.end()) {
    // undistortion + rectification
    Eigen::Matrix<double, 2, 1> coor;
    if (bUndistortEvents)
      coor =
          camPtr->getRectifiedUndistortedCoordinate((*it_tmp)->x, (*it_tmp)->y);
    else
      coor = Eigen::Matrix<double, 2, 1>((*it_tmp)->x, (*it_tmp)->y);

    // assign
    int xcoor = std::floor(coor(0));
    int ycoor = std::floor(coor(1));

    for (int dy = -dilate_radius; dy <= dilate_radius; dy++)
      for (int dx = -dilate_radius; dx <= dilate_radius; dx++) {
        int x = xcoor + dx;
        int y = ycoor + dy;

        if (x < 0 || x >= col || y < 0 || y >= row) {
        } else {
          edgeMap.at<uchar>(y, x) = 255;
          vEdgeletCoordinates.emplace_back((size_t)x, (size_t)y);
        }
      }
    it_tmp++;
  }
}

void esvo2_Mapping::createDenoisingMask(std::vector<Event *> &vAllEventsPtr,
                                        cv::Mat &mask, size_t row, size_t col) {
  cv::Mat eventMap;
  visualizor_.plot_eventMap(vAllEventsPtr, eventMap, row, col);
  cv::medianBlur(eventMap, mask, 3);
}

void esvo2_Mapping::extractDenoisedEvents(std::vector<Event *> &vCloseEventsPtr,
                                          std::vector<Event *> &vEdgeEventsPtr,
                                          cv::Mat &mask, size_t maxNum) {
  vEdgeEventsPtr.reserve(vCloseEventsPtr.size());
  for (size_t i = 0; i < vCloseEventsPtr.size(); i++) {
    if (vEdgeEventsPtr.size() >= maxNum)
      break;
    if (vCloseEventsPtr[i]->x > mask.cols ||
        vCloseEventsPtr[i]->y > mask.rows || vCloseEventsPtr[i]->x < 0 ||
        vCloseEventsPtr[i]->y < 0)
      continue;
    size_t x = vCloseEventsPtr[i]->x;
    size_t y = vCloseEventsPtr[i]->y;
    if (mask.at<uchar>(y, x) == 255)
      vEdgeEventsPtr.push_back(vCloseEventsPtr[i]);
  }
}

void esvo2_Mapping::getReprojection(
    std::vector<EventMatchPair> &vEMP, Eigen::Matrix4d T_last_now,
    std::vector<Event *> &vDenoisedEventsPtr_left_dy) {
  for (auto event : vDenoisedEventsPtr_left_dy) {
    EventMatchPair emp;
    emp.x_left_ << camSysPtr_->cam_left_ptr_->getRectifiedUndistortedCoordinate(
        event->x, event->y);
    emp.invDepth_ = 0.1;
    emp.lr_depth = 1 / emp.invDepth_;
    emp.x_left_raw_ << event->x, event->y;
    vEMP.push_back(emp);
  }
  Eigen::Matrix3d R_last_now = T_last_now.block(0, 0, 3, 3);
  Eigen::Vector3d t_last_now = T_last_now.block(0, 3, 3, 1);
  Eigen::Matrix3d K = camSysPtr_->cam_left_ptr_->P_.block(0, 0, 3, 3);
  Eigen::Matrix3d K_inv = K.inverse();
  for (int i = 0; i < vEMP.size(); i++) {
    Eigen::Vector3d x_last, x_now;
    x_now << vEMP[i].x_left_(0), vEMP[i].x_left_(1), 1;
    double depth = 1 / vEMP[i].invDepth_;
    x_last = (R_last_now * (depth * K_inv * x_now) + t_last_now);
    x_last = K * x_last / x_last(2);
    vEMP[i].x_last_ << x_last(0), x_last(1);
  }
}

void esvo2_Mapping::selectPoint() {

  // get gradient map
  cv::Mat events_map =
      cv::Mat::zeros(TS_obs_ptr_->second.cvImagePtr_left_->image.size(), CV_8U);

  vDenoisedEventsPtr_left_dx_.clear();
  vDenoisedEventsPtr_left_dy_.clear();
  vDenoisedEventsPtr_left_dx_.reserve(PROCESS_EVENT_NUM_);
  vDenoisedEventsPtr_left_dy_.reserve(PROCESS_EVENT_NUM_);

  // divide events by dx_dy
  for (auto event : vDenoisedEventsPtr_left_) {
    int redundant = 4;
    Eigen::Vector2d x;
    x << event->x, event->y;
    if (x(1) - redundant < redundant ||
        x(1) + redundant >= TS_obs_ptr_->second.cvImagePtr_left_->image.rows ||
        x(0) - redundant < redundant ||
        x(0) + redundant >= TS_obs_ptr_->second.cvImagePtr_left_->image.cols) {
      continue;
    }
    x = camSysPtr_->cam_left_ptr_->getRectifiedUndistortedCoordinate(event->x,
                                                                     event->y);
    if (x(1) - redundant < redundant ||
        x(1) + redundant >= TS_obs_ptr_->second.cvImagePtr_left_->image.rows ||
        x(0) - redundant < redundant ||
        x(0) + redundant >= TS_obs_ptr_->second.cvImagePtr_left_->image.cols) {
      continue;
    }
    double dx =
        abs(TS_obs_ptr_->second.dTS_negative_du_left_((int)x(1), (int)x(0)));
    double dy =
        abs(TS_obs_ptr_->second.dTS_negative_dv_left_((int)x(1), (int)x(0)));
    dx = ((dx <= 0.01) ? 0.1 : dx);
    dy = ((dy <= 0.01) ? 0.1 : dy);
    double dx_dy = (double)dx / (double)dy;
    if (dx_dy < eta_for_select_points_) {
      vDenoisedEventsPtr_left_dy_.push_back(event);
    }
    events_map.at<uchar>(x(1), x(0)) = 255;
    if (dx_dy >= eta_for_select_points_) {
      vDenoisedEventsPtr_left_dx_.push_back(event);
    }
  }
}

bool esvo2_Mapping::getIMUInterval(
    double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
    vector<pair<double, Eigen::Vector3d>> &gyrVector) {
  if (accBuf.empty()) {
    LOG(ERROR) << "not receive imu data";
    return false;
  }
  if (t1 <= accBuf.back().first) {
    while (accBuf.front().first <= t0) {
      accBuf.pop();
      gyrBuf.pop();
    }
    while (accBuf.front().first < t1) {
      accVector.push_back(accBuf.front());
      accBuf.pop();
      gyrVector.push_back(gyrBuf.front());
      gyrBuf.pop();
    }
    accVector.push_back(accBuf.front());
    gyrVector.push_back(gyrBuf.front());
  } else
    return false;
  return true;
}

void esvo2_Mapping::initFirstIMUPose(
    vector<pair<double, Eigen::Vector3d>> &accVector) {
  LOG(INFO) << "init first imu pose";
  initFirstPoseFlag = true;
  // return;
  Eigen::Vector3d averAcc(0, 0, 0);
  int n = (int)accVector.size();
  for (size_t i = 0; i < accVector.size(); i++) {
    averAcc = averAcc + accVector[i].second;
  }
  averAcc = averAcc / n;
  Eigen::Matrix3d R0 = Utility::g2R(averAcc);
  double yaw = Utility::R2ypr(R0).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  BackendOpt_.Rs[0] = R0;
}

void esvo2_Mapping::processIMU(double t, double dt,
                               const Eigen::Vector3d &linear_acceleration,
                               const Eigen::Vector3d &angular_velocity) {
  if (!first_imu) {
    first_imu = true;
    BackendOpt_.acc_0 = linear_acceleration;
    acc_0 = linear_acceleration;
    BackendOpt_.gyr_0 = angular_velocity;
    gyr_0 = angular_velocity;
  }

  if (!BackendOpt_.pre_integrations[BackendOpt_.frame_count]) {
    BackendOpt_.pre_integrations[BackendOpt_.frame_count] = new IntegrationBase{
        BackendOpt_.acc_0, BackendOpt_.gyr_0,
        BackendOpt_.Bas[BackendOpt_.frame_count],
        BackendOpt_.Bgs[BackendOpt_.frame_count], BackendOpt_.g_optimal};
  }
  if (BackendOpt_.frame_count != 0) {
    BackendOpt_.pre_integrations[BackendOpt_.frame_count]->push_back(
        dt, linear_acceleration, angular_velocity);
  }
  BackendOpt_.acc_0 = linear_acceleration;
  BackendOpt_.gyr_0 = angular_velocity;
  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
}
} // namespace esvo2_core