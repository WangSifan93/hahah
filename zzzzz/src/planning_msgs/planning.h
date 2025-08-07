/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <string>
#include <vector>

#include "apps/planning/src/common/math/polynomial_fit/polynomial_fit.h"
#include "geometry.h"
#include "header.h"
#include "messages/mapfusion/all_map.pb.h"
#include "messages/planning/planning.pb.h"
#include "messages/planning/planning_debug.pb.h"
#include "pnc_point.h"

namespace zark {
namespace planning {

// Struct
enum class JucType {
  UNKNOWN = 0,
  IN_ROAD = 1,
  CROSS_ROAD = 2,
  FORK_ROAD = 3,
  MAIN_SIDE = 4,
  DEAD_END = 5
};

enum class RightOfWayStatus { UNPROTECTED = 0, PROTECTED = 1 };

class EStop {
 public:
  // Default constructor
  EStop() = default;

  // Getter functions
  bool is_estop() const { return is_estop_; }
  const std::string& reason() const { return reason_; }

  // Setter functions
  void set_is_estop(bool is_estop) { is_estop_ = is_estop; }
  void set_reason(const std::string& reason) { reason_ = reason; }

 private:
  bool is_estop_;
  std::string reason_;
};

class TaskStats {
 public:
  // Default constructor
  TaskStats() = default;

  // Getter functions
  const std::string& name() const { return name_; }
  double time_ms() const { return time_ms_; }

  // Setter functions
  void set_name(const std::string& name) { name_ = name; }
  void set_time_ms(double time_ms) { time_ms_ = time_ms; }

 private:
  std::string name_;
  double time_ms_;
};

class LatencyStats {
 public:
  // Default constructor
  LatencyStats() = default;

  // Getter functions
  double total_time_ms() const { return total_time_ms_; }
  const std::vector<TaskStats>& task_stats() const { return task_stats_; }
  double init_frame_time_ms() const { return init_frame_time_ms_; }

  // Setter functions
  void set_total_time_ms(double total_time_ms) {
    total_time_ms_ = total_time_ms;
  }
  void set_task_stats(const std::vector<TaskStats>& task_stats) {
    task_stats_ = task_stats;
  }
  void set_init_frame_time_ms(double init_frame_time_ms) {
    init_frame_time_ms_ = init_frame_time_ms;
  }

  TaskStats* add_task_stats() {
    TaskStats task_stats;
    task_stats_.emplace_back(task_stats);
    return &task_stats_.back();
  }

  void MergeFrom(const LatencyStats& stats) {
    // TODO add task_stats merge fuvtion
    total_time_ms_ = stats.total_time_ms();
    init_frame_time_ms_ = stats.init_frame_time_ms();
  }

  std::string DebugString() const { return "LatencyStats"; }

 private:
  double total_time_ms_;
  std::vector<TaskStats> task_stats_;
  double init_frame_time_ms_;
};

class RSSInfo {
 public:
  // Default constructor
  RSSInfo() = default;

  // Getter functions
  bool is_rss_safe() const { return is_rss_safe_; }
  double cur_dist_lon() const { return cur_dist_lon_; }
  double rss_safe_dist_lon() const { return rss_safe_dist_lon_; }
  double acc_lon_range_minimum() const { return acc_lon_range_minimum_; }
  double acc_lon_range_maximum() const { return acc_lon_range_maximum_; }
  double acc_lat_left_range_minimum() const {
    return acc_lat_left_range_minimum_;
  }
  double acc_lat_left_range_maximum() const {
    return acc_lat_left_range_maximum_;
  }
  double acc_lat_right_range_minimum() const {
    return acc_lat_right_range_minimum_;
  }
  double acc_lat_right_range_maximum() const {
    return acc_lat_right_range_maximum_;
  }

  // Setter functions
  void set_is_rss_safe(bool is_rss_safe) { is_rss_safe_ = is_rss_safe; }
  void set_cur_dist_lon(double cur_dist_lon) { cur_dist_lon_ = cur_dist_lon; }
  void set_rss_safe_dist_lon(double rss_safe_dist_lon) {
    rss_safe_dist_lon_ = rss_safe_dist_lon;
  }
  void set_acc_lon_range_minimum(double acc_lon_range_minimum) {
    acc_lon_range_minimum_ = acc_lon_range_minimum;
  }
  void set_acc_lon_range_maximum(double acc_lon_range_maximum) {
    acc_lon_range_maximum_ = acc_lon_range_maximum;
  }
  void set_acc_lat_left_range_minimum(double acc_lat_left_range_minimum) {
    acc_lat_left_range_minimum_ = acc_lat_left_range_minimum;
  }
  void set_acc_lat_left_range_maximum(double acc_lat_left_range_maximum) {
    acc_lat_left_range_maximum_ = acc_lat_left_range_maximum;
  }
  void set_acc_lat_right_range_minimum(double acc_lat_right_range_minimum) {
    acc_lat_right_range_minimum_ = acc_lat_right_range_minimum;
  }
  void set_acc_lat_right_range_maximum(double acc_lat_right_range_maximum) {
    acc_lat_right_range_maximum_ = acc_lat_right_range_maximum;
  }

 private:
  bool is_rss_safe_;
  double cur_dist_lon_;
  double rss_safe_dist_lon_;
  double acc_lon_range_minimum_;
  double acc_lon_range_maximum_;
  double acc_lat_left_range_minimum_;
  double acc_lat_left_range_maximum_;
  double acc_lat_right_range_minimum_;
  double acc_lat_right_range_maximum_;
};

class CriticalRegion {
 public:
  // Default constructor
  CriticalRegion() = default;

  // Getter function
  const std::vector<::common::Polygon>& region() const { return region_; }

  // Setter function
  void set_region(const std::vector<::common::Polygon>& region) {
    region_ = region;
  }

 private:
  std::vector<::common::Polygon> region_;
};

class ADCTrajectory {
 public:
  // Default constructor
  ADCTrajectory() = default;

  // Getter functions
  const ::common::Header header() const { return header_; }

  const EStop& estop() const { return estop_; }

  bool is_replan() const { return is_replan_; }

  const std::vector<::common::TrajectoryPoint>& trajectory_point() const {
    return trajectory_point_;
  }

  const LatencyStats& latency_stats() const { return latency_stats_; }

  const std::vector<zark::hdmap::Id>& lane_id() const { return lane_id_; }

  const std::string& replan_reason() const { return replan_reason_; }
  const std::vector<zark::hdmap::Id>& target_lane_id() const {
    return target_lane_id_;
  }

  // Setter functions
  void set_header(const ::common::Header& header) { header_ = header; }

  void set_estop(const EStop& estop) { estop_ = estop; }

  void set_is_replan(bool is_replan) { is_replan_ = is_replan; }

  void set_trajectory_point(
      const std::vector<::common::TrajectoryPoint>& trajectory_point) {
    trajectory_point_ = trajectory_point;
  }
  void set_latency_stats(const LatencyStats& latency_stats) {
    latency_stats_ = latency_stats;
  }
  void set_lane_id(const std::vector<zark::hdmap::Id>& lane_id) {
    lane_id_ = lane_id;
  }
  void add_lane_id(const zark::hdmap::Id& id) { lane_id_.emplace_back(id); }

  void set_target_lane_id(const std::vector<zark::hdmap::Id>& target_lane_id) {
    target_lane_id_ = target_lane_id;
  }
  void add_target_lane_id(const zark::hdmap::Id& id) {
    target_lane_id_.emplace_back(id);
  }

  void mutable_trajectory_point(  // need reviewer check!
      const std::vector<::common::TrajectoryPoint>& points) {
    this->trajectory_point_ = points;
  }
  LatencyStats* mutable_latency_stats() { return &latency_stats_; }
  ::common::Header* mutable_header() { return &header_; }
  EStop* mutable_estop() { return &estop_; }

  void clear_trajectory_point() { trajectory_point_.clear(); }

  ::common::TrajectoryPoint* add_trajectory_point() {
    ::common::TrajectoryPoint point;
    trajectory_point_.emplace_back(point);
    return &trajectory_point_.back();
  }

  void CopyFrom(const ADCTrajectory& other) { *this = other; }

  const PlanningStatus& planning_status() const { return planning_status_; }

  void set_planning_status(const PlanningStatus& planning_status) {
    planning_status_ = planning_status;
  }

  const PolynomialFit::TrajPolynomial traj_polyfit_out() const {
    return traj_polyfit_out_;
  }

  PolynomialFit::TrajPolynomial* mutable_traj_polyfit_out() {
    return &traj_polyfit_out_;
  }

  void set_traj_polyfit_out(
      const PolynomialFit::TrajPolynomial& traj_polyfit_out) {
    traj_polyfit_out_ = traj_polyfit_out;
  }

  const PlanningDebugInfo& planning_debug_info() const {
    return planning_debug_info_;
  }
  PlanningDebugInfo* mutable_planning_debug_info() {
    return &planning_debug_info_;
  }

 private:
  ::common::Header header_;
  EStop estop_;
  bool is_replan_;
  std::string replan_reason_;
  std::vector<::common::TrajectoryPoint> trajectory_point_;

  PolynomialFit::TrajPolynomial traj_polyfit_out_;
  LatencyStats latency_stats_;
  std::vector<zark::hdmap::Id> lane_id_;
  std::vector<zark::hdmap::Id> target_lane_id_;
  PlanningDebugInfo planning_debug_info_;
  PlanningStatus planning_status_;
};

}  // namespace planning
}  // namespace zark
