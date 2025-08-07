#pragma once

#include "geometry.h"

namespace zark {
namespace planning {
namespace common {
class VehicleState {
 public:
  enum GearPosition {
    GEAR_NEUTRAL = 0,
    GEAR_DRIVE = 1,
    GEAR_REVERSE = 2,
    GEAR_PARKING = 3,
    GEAR_LOW = 4,
    GEAR_INVALID = 5,
    GEAR_NONE = 6,
  };

  enum DrivingMode {
    COMPLETE_MANUAL = 0,  // human drive
    COMPLETE_AUTO_DRIVE = 1,
    AUTO_STEER_ONLY = 2,  // only steer
    AUTO_SPEED_ONLY = 3,  // include throttle and brake
    // security mode when manual intervention happens, only response status
    EMERGENCY_MODE = 4,
  };

  const double x() const { return x_; }
  const double y() const { return y_; }
  const double z() const { return z_; }
  const double roll() const { return roll_; }
  const double pitch() const { return pitch_; }
  const double yaw() const { return yaw_; }
  const double linear_velocity() const { return linear_velocity_; }
  const double heading() const { return heading_; }
  const double timestamp() const { return timestamp_; }
  const int64_t nano_timestamp() const { return nano_timestamp_; }
  const double kappa() const { return kappa_; }
  const double angular_velocity() const { return angular_velocity_; }
  const double linear_acceleration() const { return linear_acceleration_; }
  const double steering_percentage() const { return steering_percentage_; }
  const double front_wheel_angle() const { return front_wheel_angle_; }
  const double front_wheel_angle_rate() const {
    return front_wheel_angle_rate_;
  }
  const GearPosition gear() const { return gear_; }

  void set_x(const double x) { x_ = x; }
  void set_y(const double y) { y_ = y; }
  void set_z(const double z) { z_ = z; }
  void set_heading(const double head) { heading_ = head; }
  void set_yaw(const double data) { yaw_ = data; }
  void set_timestamp(const double t) { timestamp_ = t; }
  void set_nano_timestamp(int64_t t) { nano_timestamp_ = t; }
  void set_kappa(const double kappa) { kappa_ = kappa; }
  void set_linear_velocity(const double velocity) {
    linear_velocity_ = velocity;
  }
  void set_roll(const double input) { roll_ = input; }
  void set_pitch(const double input) { pitch_ = input; }
  void set_angular_velocity(const double velocity) {
    angular_velocity_ = velocity;
  }
  void set_linear_acceleration(const double acceleration) {
    linear_acceleration_ = acceleration;
  }
  void set_steering_percentage(const double steering_percentage) {
    steering_percentage_ = steering_percentage;
  }
  void set_front_wheel_angle(const double front_wheel_angle) {
    front_wheel_angle_ = front_wheel_angle;
  }
  void set_front_wheel_angle_rate(const double front_wheel_angle_rate) {
    front_wheel_angle_rate_ = front_wheel_angle_rate;
  }
  void set_driving_mode(const DrivingMode mode) { driving_mode_ = mode; }
  void set_gear(const GearPosition gear) { gear_ = gear; }

  const DrivingMode driving_mode() const { return driving_mode_; }
  const std::string DebugString() const { return "DebugString!"; }

 private:
  double x_;          // [default = 0.0];
  double y_;          //[default = 0.0];
  double z_;          // [default = 0.0];
  double timestamp_;  // [default = 0.0];
  int64_t nano_timestamp_;
  double roll_;                 // [default = 0.0];
  double pitch_;                // [default = 0.0];
  double yaw_;                  // [default = 0.0];
  double heading_;              // [default = 0.0];
  double kappa_;                // [default = 0.0];
  double linear_velocity_;      // [default = 0.0];
  double angular_velocity_;     // [default = 0.0];
  double linear_acceleration_;  // [default = 0.0];
  double steering_percentage_;
  double front_wheel_angle_;       // [rad]
  double front_wheel_angle_rate_;  // [rad/s]
  GearPosition gear_;
  DrivingMode driving_mode_;
};
}  // namespace common
}  // namespace planning
}  // namespace zark
