#ifndef __UAV_UTILS_H
#define __UAV_UTILS_H

#include <rclcpp/rclcpp.hpp>

#include <uav_utils/converters.h>
#include <uav_utils/geometry_utils.h>

namespace uav_utils
{

/* 判断 value 是否在 [low, high] 范围内 */
template <typename T, typename T2>
bool in_range(T value, const T2& low, const T2& high) {
  RCLCPP_ASSERT(rclcpp::get_logger("uav_utils"), low < high, 
                "%f < %f?", static_cast<double>(low), static_cast<double>(high));
  return (low <= value) && (value <= high);
}

/* 判断 value 是否在 [-limit, limit] 范围内 */
template <typename T, typename T2>
bool in_range(T value, const T2& limit) {
  RCLCPP_ASSERT(rclcpp::get_logger("uav_utils"), limit > 0, 
                "%f > 0?", static_cast<double>(limit));
  return in_range(value, -limit, limit);
}

template <typename T, typename T2>
void limit_range(T& value, const T2& low, const T2& high) {
  RCLCPP_ASSERT(rclcpp::get_logger("uav_utils"), low < high, 
                "%f < %f?", static_cast<double>(low), static_cast<double>(high));
  if (value < low) {
    value = low;
  }

  if (value > high) {
    value = high;
  }

  return;
}

template <typename T, typename T2>
void limit_range(T& value, const T2& limit) {
  RCLCPP_ASSERT(rclcpp::get_logger("uav_utils"), limit > 0, 
                "%f > 0?", static_cast<double>(limit));
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
}

#endif