/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <string>
#include <vector>

#include "geometry.h"
#include "header.h"

namespace zark {
namespace planning {
namespace common {
class Transform {
 public:
  // Default constructor
  Transform() = default;

  // Getter function for source_frame
  std::string source_frame() const { return source_frame_; }

  // Setter function for source_frame
  void set_source_frame(const std::string& source_frame) {
    source_frame_ = source_frame;
  }

  // Getter function for target_frame
  std::string target_frame() const { return target_frame_; }

  // Setter function for target_frame
  void set_target_frame(const std::string& target_frame) {
    target_frame_ = target_frame;
  }

  // Getter function for translation
  ::common::Point3D translation() const { return translation_; }

  // Setter function for translation
  void set_translation(const ::common::Point3D& translation) {
    translation_ = translation;
  }

  // Getter function for rotation
  std::string rotation() const { return rotation_; }

  // Setter function for rotation
  void set_rotation(const std::string& rotation) { rotation_ = rotation; }

 private:
  std::string source_frame_;
  std::string target_frame_;
  ::common::Point3D translation_;
  std::string rotation_;
};

class Extrinsics {
 public:
  // Default constructor
  Extrinsics() = default;

  // Getter function for transform
  const std::vector<Transform>& transform() const { return transform_; }

  // Setter function for transform
  void set_transform(const std::vector<Transform>& transform) {
    transform_ = transform;
  }

  /// @brief add transform
  /// @param transform
  void add_transform(Transform transform) {
    transform_.emplace_back(transform);
  }

  void clear_transform() { transform_.clear(); }

 private:
  std::vector<Transform> transform_;
};

// Vehicle parameters shared among several modules.
// By default, all are measured with the SI units (meters, meters per second,
// etc.).

enum VehicleBrand {
  LINCOLN_MKZ = 0,
  GEM = 1,
  LEXUS = 2,
  TRANSIT = 3,
  GE3 = 4,
  WEY = 5,
  ZHONGYUN = 6,
  CH = 7,
  DKIT = 8,
  NEOLIX = 9,
};

class LatencyParam {
 public:
  // Default constructor
  LatencyParam() = default;

  // Setter functions
  void set_dead_time(double dead_time) { dead_time_ = dead_time; }
  void set_rise_time(double rise_time) { rise_time_ = rise_time; }
  void set_peak_time(double peak_time) { peak_time_ = peak_time; }
  void set_settling_time(double settling_time) {
    settling_time_ = settling_time;
  }

  // Getter functions
  double dead_time() const { return dead_time_; }
  double rise_time() const { return rise_time_; }
  double peak_time() const { return peak_time_; }
  double settling_time() const { return settling_time_; }

 private:
  double dead_time_;
  double rise_time_;
  double peak_time_;
  double settling_time_;
};

class VehicleParam {
 public:
  // Default constructor
  VehicleParam() = default;

  // Getter and setter functions for brand
  const VehicleBrand& brand() const { return brand_; }
  void set_brand(const VehicleBrand& brand) { brand_ = brand; }
  VehicleBrand get_brand() { return brand_; }

  // Getter and setter functions for front_edge_to_center
  double front_edge_to_center() const { return front_edge_to_center_; }
  void set_front_edge_to_center(double front_edge_to_center) {
    front_edge_to_center_ = front_edge_to_center;
  }

  // Getter and setter functions for back_edge_to_center
  double back_edge_to_center() const { return back_edge_to_center_; }
  void set_back_edge_to_center(double back_edge_to_center) {
    back_edge_to_center_ = back_edge_to_center;
  }

  // Getter and setter functions for left_edge_to_center
  double left_edge_to_center() const { return left_edge_to_center_; }
  void set_left_edge_to_center(double left_edge_to_center) {
    left_edge_to_center_ = left_edge_to_center;
  }

  // Getter and setter functions for right_edge_to_center
  double right_edge_to_center() const { return right_edge_to_center_; }
  void set_right_edge_to_center(double right_edge_to_center) {
    right_edge_to_center_ = right_edge_to_center;
  }

  // Getter and setter functions for length
  double length() const { return length_; }
  void set_length(double length) { length_ = length; }

  // Getter and setter functions for width
  double width() const { return width_; }
  void set_width(double width) { width_ = width; }

  // Getter and setter functions for height
  double height() const { return height_; }
  void set_height(double height) { height_ = height; }

  double rear_axle_to_cg() const { return rear_axle_to_cg_; }
  void set_rear_axle_to_cg(const double value) { rear_axle_to_cg_ = value; }

  // Getter and setter functions for min_turn_radius
  double min_turn_radius() const { return min_turn_radius_; }
  void set_min_turn_radius(double min_turn_radius) {
    min_turn_radius_ = min_turn_radius;
  }

  // Getter and setter functions for max_acceleration
  double max_acceleration() const { return max_acceleration_; }
  void set_max_acceleration(double max_acceleration) {
    max_acceleration_ = max_acceleration;
  }

  // Getter and setter functions for max_deceleration
  double max_deceleration() const { return max_deceleration_; }
  void set_max_deceleration(double max_deceleration) {
    max_deceleration_ = max_deceleration;
  }

  // Getter and setter functions for max_steer_angle
  double max_steer_angle() const { return max_steer_angle_; }
  void set_max_steer_angle(double max_steer_angle) {
    max_steer_angle_ = max_steer_angle;
  }

  // Getter and setter functions for max_steer_angle_rate
  double max_steer_angle_rate() const { return max_steer_angle_rate_; }
  void set_max_steer_angle_rate(double max_steer_angle_rate) {
    max_steer_angle_rate_ = max_steer_angle_rate;
  }

  // Getter and setter functions for min_steer_angle_rate
  double min_steer_angle_rate() const { return min_steer_angle_rate_; }
  void set_min_steer_angle_rate(double min_steer_angle_rate) {
    min_steer_angle_rate_ = min_steer_angle_rate;
  }

  // Getter and setter functions for steer_ratio
  double steer_ratio() const { return steer_ratio_; }
  void set_steer_ratio(double steer_ratio) { steer_ratio_ = steer_ratio; }

  // Getter and setter functions for wheel_base
  double wheel_base() const { return wheel_base_; }
  void set_wheel_base(double wheel_base) { wheel_base_ = wheel_base; }

  // Getter and setter functions for wheel_rolling_radius
  double wheel_rolling_radius() const { return wheel_rolling_radius_; }
  void set_wheel_rolling_radius(double wheel_rolling_radius) {
    wheel_rolling_radius_ = wheel_rolling_radius;
  }

  double front_wheel_corner_stiffness() const {
    return front_wheel_corner_stiffness_;
  }
  void set_front_wheel_corner_stiffness(const double value) {
    front_wheel_corner_stiffness_ = value;
  }

  double rear_wheel_corner_stiffness() const {
    return rear_wheel_corner_stiffness_;
  }
  void set_rear_wheel_corner_stiffness(const double value) {
    rear_wheel_corner_stiffness_ = value;
  }

  double mass() const { return mass_; }
  void set_mass(const double value) { mass_ = value; }

  double moment_of_inertia() const { return moment_of_inertia_; }
  void set_moment_of_inertia(const double value) { moment_of_inertia_ = value; }

  // Getter and setter functions for max_abs_speed_when_stopped
  float max_abs_speed_when_stopped() const {
    return max_abs_speed_when_stopped_;
  }
  void set_max_abs_speed_when_stopped(float max_abs_speed_when_stopped) {
    max_abs_speed_when_stopped_ = max_abs_speed_when_stopped;
  }

  // Getter and setter functions for steering_latency_param
  const LatencyParam& steering_latency_param() const {
    return steering_latency_param_;
  }
  void set_steering_latency_param(const LatencyParam& steering_latency_param) {
    steering_latency_param_ = steering_latency_param;
  }
  LatencyParam get_steering_latency_param() { return steering_latency_param_; }

  // Getter and setter functions for throttle_latency_param
  const LatencyParam& throttle_latency_param() const {
    return throttle_latency_param_;
  }
  void set_throttle_latency_param(const LatencyParam& throttle_latency_param) {
    throttle_latency_param_ = throttle_latency_param;
  }
  LatencyParam get_throttle_latency_param() { return throttle_latency_param_; }

  // Getter and setter functions for brake_latency_param
  const LatencyParam& brake_latency_param() const {
    return brake_latency_param_;
  }
  void set_brake_latency_param(const LatencyParam& brake_latency_param) {
    brake_latency_param_ = brake_latency_param;
  }
  LatencyParam get_brake_latency_param() { return brake_latency_param_; }

  std::string DebugString() const { return "DebugString!"; }

 private:
  VehicleBrand brand_;
  double front_edge_to_center_;  // [m]
  double back_edge_to_center_;   // [m]
  double left_edge_to_center_;   // [m]
  double right_edge_to_center_;  // [m]
  double length_ = 4.0;          // [m]
  double width_ = 1.8;           // [m]
  double height_ = 1.6;          // [m]
  double rear_axle_to_cg_;       // [m]
  double min_turn_radius_;       // [m]
  double max_acceleration_;      //[m/s^2]
  double max_deceleration_;      //[m/s^2]
  double max_steer_angle_;       // [rad]
  double max_steer_angle_rate_;  // [rad/s]
  double min_steer_angle_rate_;  // [rad/s]
  double steer_ratio_;
  double wheel_base_;                    // [m]
  double wheel_rolling_radius_;          // [m]
  double front_wheel_corner_stiffness_;  // [N/rad]
  double rear_wheel_corner_stiffness_;   // [N/rad]
  double mass_;                          // [kg]
  double moment_of_inertia_;             // [kg·m^2]
  float max_abs_speed_when_stopped_;     // [m/s]
  LatencyParam steering_latency_param_;
  LatencyParam throttle_latency_param_;
  LatencyParam brake_latency_param_;
};

class VehicleConfig {
 public:
  // Default constructor
  VehicleConfig() = default;

  // Getter and setter functions for header
  const ::common::Header& header() const { return header_; }
  void set_header(const ::common::Header& header) { header_ = header; }

  // Getter and setter functions for vehicle_param
  const VehicleParam& vehicle_param() const { return vehicle_param_; }
  void set_vehicle_param(const VehicleParam& vehicle_param) {
    vehicle_param_ = vehicle_param;
  }

  // Getter and setter functions for extrinsics
  const Extrinsics& extrinsics() const { return extrinsics_; }
  void set_extrinsics(const Extrinsics& extrinsics) {
    extrinsics_ = extrinsics;
  }

 private:
  ::common::Header header_;
  VehicleParam vehicle_param_;
  Extrinsics extrinsics_;
};

}  // namespace common

}  // namespace planning
}  // namespace zark
