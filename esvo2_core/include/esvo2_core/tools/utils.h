#ifndef ESVO2_CORE_TOOLS_UTILS_H
#define ESVO2_CORE_TOOLS_UTILS_H

#include <Eigen/Eigen>
#include <iostream>
#include <sys/stat.h>

#include <cv_bridge/cv_bridge.hpp>

#include <kindr/minimal/quat-transformation.h>

#include <esvo2_core/core/Event.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <esvo2_core/container/DepthPoint.h>
#include <esvo2_core/container/SmartGrid.h>
#include <esvo2_core/tools/TicToc.h>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
namespace esvo2_core {
namespace tools {
// TUNE this according to your platform's computational capability.
#define NUM_THREAD_TRACKING 1
#define NUM_THREAD_MAPPING 4

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using RefPointCloudMap = std::map<timestamp_t, PointCloud::Ptr>;

using Transformation = kindr::minimal::QuatTransformation;

inline static std::vector<Event *>::iterator
EventVecPtr_lower_bound(std::vector<Event *> &vEventPtr, timestamp_t t) {
  return std::lower_bound(
      vEventPtr.begin(), vEventPtr.end(), t,
      [](const Event *e, timestamp_t t) { return e->ts < t; });
}

using EventQueue = std::deque<Event>;
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

using StampTransformationMap = std::map<timestamp_t, tools::Transformation>;
inline static StampTransformationMap::iterator
StampTransformationMap_lower_bound(StampTransformationMap &stm, timestamp_t t) {
  return std::lower_bound(
      stm.begin(), stm.end(), t,
      [](const std::pair<timestamp_t, tools::Transformation> &st,
         timestamp_t t) { return st.first < t; });
}

/******************* Used by Block Match ********************/
static inline void meanStdDev(Eigen::MatrixXd &patch, double &mean,
                              double &sigma) {
  double numElement = (patch.rows() * patch.cols());
  mean = patch.array().sum() / numElement;
  Eigen::MatrixXd sub = patch.array() - mean;
  sigma = sqrt((sub.array() * sub.array()).sum() / numElement) + 1e-6;
}

static inline void normalizePatch(Eigen::MatrixXd &patch_src,
                                  Eigen::MatrixXd &patch_dst) {
  double mean = 0;
  double sigma = 0;
  meanStdDev(patch_src, mean, sigma);
  sigma = 1.0 / sigma;
  patch_dst = (patch_src.array() - mean) * sigma;
}

// recursively create a directory
static inline void _mkdir(const char *dir) {
  char tmp[256];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp), "%s", dir);
  len = strlen(tmp);
  if (tmp[len - 1] == '/')
    tmp[len - 1] = 0;
  for (p = tmp + 1; *p; p++)
    if (*p == '/') {
      *p = 0;
      mkdir(tmp, S_IRWXU);
      *p = '/';
    }
  mkdir(tmp, S_IRWXU);
}

} // namespace tools
} // namespace esvo2_core
#endif // ESVO2_CORE_TOOLS_UTILS_H
