#ifndef ESVO2_CORE_PARAMS_HELPER_H
#define ESVO2_CORE_PARAMS_HELPER_H

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace esvo2_core {
namespace tools {
// inline
// bool hasParam(const std::string &name)
//{
//   return ros::param::has(name);
// }
//
// template<typename T>
// T getParam(const std::string &name, const T &defaultValue)
//{
//   T v;
//   if (ros::param::get(name, v))
//   {
//     ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
//     return v;
//   }
//   else
//     ROS_WARN_STREAM("Cannot find value for parameter: " << name << ",
//     assigning default: " << defaultValue);
//   return defaultValue;
// }
//
// template<typename T>
// T getParam(const std::string &name)
//{
//   T v;
//   if (ros::param::get(name, v))
//   {
//     ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
//     return v;
//   }
//   else
//     ROS_ERROR_STREAM("Cannot find value for parameter: " << name);
//   return T();
// }

template <typename T>
T param(rclcpp::Node *n, const std::string &name, const T &defaultValue) {
  const auto pi = n->get_node_parameters_interface();
  const auto li = n->get_node_logging_interface();
  if (pi->has_parameter(name)) // parameter has been declared
  {
    rclcpp::Parameter p;
    pi->get_parameter(name, p);
    T v = p.get_value<T>();
    RCLCPP_INFO_STREAM(li->get_logger(),
                       "Found parameter: " << name << ", value: " << v);
    return v;
  } else {
    const rclcpp::ParameterValue def(defaultValue);
    pi->declare_parameter(name, def);
    RCLCPP_WARN_STREAM(li->get_logger(),
                       "Cannot find value for parameter: "
                           << name << ", assigning default: " << defaultValue);
  }
  return defaultValue;
}
} // namespace tools
} // namespace esvo2_core

#endif // ESVO2_CORE_PARAMS_HELPER_H
