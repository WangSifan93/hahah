/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <string>

namespace zark {
namespace planning {
// Struct
class PlanningSemanticMapConfig {
 public:
  // Default constructor
  PlanningSemanticMapConfig() = default;

  // Getter functions
  double resolution() const { return resolution_; }
  int32_t height() const { return height_; }
  int32_t width() const { return width_; }
  int32_t ego_idx_x() const { return ego_idx_x_; }
  int32_t ego_idx_y() const { return ego_idx_y_; }
  double max_rand_delta_phi() const { return max_rand_delta_phi_; }
  double max_ego_future_horizon() const { return max_ego_future_horizon_; }
  double max_ego_past_horizon() const { return max_ego_past_horizon_; }
  double max_obs_future_horizon() const { return max_obs_future_horizon_; }
  double max_obs_past_horizon() const { return max_obs_past_horizon_; }
  int32_t base_map_padding() const { return base_map_padding_; }
  double city_driving_max_speed() const { return city_driving_max_speed_; }

  // Setter functions
  void set_resolution(double resolution) { resolution_ = resolution; }
  void set_height(int32_t height) { height_ = height; }
  void set_width(int32_t width) { width_ = width; }
  void set_ego_idx_x(int32_t ego_idx_x) { ego_idx_x_ = ego_idx_x; }
  void set_ego_idx_y(int32_t ego_idx_y) { ego_idx_y_ = ego_idx_y; }
  void set_max_rand_delta_phi(double max_rand_delta_phi) {
    max_rand_delta_phi_ = max_rand_delta_phi;
  }
  void set_max_ego_future_horizon(double max_ego_future_horizon) {
    max_ego_future_horizon_ = max_ego_future_horizon;
  }
  void set_max_ego_past_horizon(double max_ego_past_horizon) {
    max_ego_past_horizon_ = max_ego_past_horizon;
  }
  void set_max_obs_future_horizon(double max_obs_future_horizon) {
    max_obs_future_horizon_ = max_obs_future_horizon;
  }
  void set_max_obs_past_horizon(double max_obs_past_horizon) {
    max_obs_past_horizon_ = max_obs_past_horizon;
  }
  void set_base_map_padding(int32_t base_map_padding) {
    base_map_padding_ = base_map_padding;
  }
  void set_city_driving_max_speed(double city_driving_max_speed) {
    city_driving_max_speed_ = city_driving_max_speed;
  }

 private:
  double resolution_;  // Map resolution
  int32_t height_;     // Map height
  int32_t width_;      // Map width
  int32_t ego_idx_x_;  // Ego vehicle index (x-coordinate)
  int32_t ego_idx_y_;  // Ego vehicle index (y-coordinate)
  double
      max_rand_delta_phi_;  // Maximum random delta for semantic map generation
  double max_ego_future_horizon_;  // Maximum ego vehicle future horizon
  double max_ego_past_horizon_;    // Maximum ego vehicle past horizon
  double max_obs_future_horizon_;  // Maximum obstacle future horizon
  double max_obs_past_horizon_;    // Maximum obstacle past horizon
  int32_t base_map_padding_;       // Base map padding
  double city_driving_max_speed_;  // Maximum speed for city driving
};

}  // namespace planning
}  // namespace zark
