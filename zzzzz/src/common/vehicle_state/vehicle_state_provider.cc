/******************************************************************************
 * Copyright 2017 The zpilot Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "apps/planning/src/common/vehicle_state/vehicle_state_provider.h"

#include <cmath>

#include "Eigen/Core"
#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/common/log.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "euler_angles_zxy.h"
#include "quaternion.h"
#include "string_util.h"

namespace zark {
namespace planning {
namespace common {

using namespace ::common;

Status VehicleStateProvider::Update(
    const zark::localization::LocalizationInfo& localization,
    const zark::ads_adptrin::ADAS_Inputs& can_data) {
  if (!ConstructExceptLinearVelocity(localization, can_data)) {
    std::string msg =
        ("Fail to update because ConstructExceptLinearVelocity error." +
         std::string("localization:\n") +
         std::string("localization.DebugString:"));
    return Status(ErrorCode::LOCALIZATION_ERROR, msg);
  }
  if (!ConstructSteerAngle(can_data)) {
    std::string msg = ("Fail to update because ConstructSteerAngle error." +
                       std::string("can:\n") + std::string("can.DebugString:"));
    return Status(ErrorCode::CANBUS_ERROR, msg);
  }
  return Status::OK();
}

bool VehicleStateProvider::ConstructSteerAngle(
    const zark::ads_adptrin::ADAS_Inputs& can_data) {
  {
    const double kDegToRad = M_PI / 180.0;
    const auto& vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    vehicle_state_.set_front_wheel_angle(
        can_data.adptrin_out_steersys().da_in_angd_strgwhlang_sg() /
        vehicle_param.steer_ratio() * kDegToRad);
    vehicle_state_.set_front_wheel_angle_rate(
        can_data.adptrin_out_steersys().da_in_wd_strgwhlanggrd_sg() /
        vehicle_param.steer_ratio() * kDegToRad);
  }
  return true;
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const zark::localization::LocalizationInfo& localization,
    const zark::ads_adptrin::ADAS_Inputs& can_data) {
  vehicle_state_.set_nano_timestamp(localization.header().timestamp_nano());
  vehicle_state_.set_x(localization.local_pose().pose().t().x());
  vehicle_state_.set_y(localization.local_pose().pose().t().y());
  vehicle_state_.set_z(localization.local_pose().pose().t().z());
  ::common::Quaternion orientation;
  orientation.set_qw(localization.local_pose().pose().q().qw());
  orientation.set_qx(localization.local_pose().pose().q().qx());
  orientation.set_qy(localization.local_pose().pose().q().qy());
  orientation.set_qz(localization.local_pose().pose().q().qz());
  double msg_heading = getYawFromQuaternion(orientation.qw(), orientation.qx(),
                                            orientation.qy(), orientation.qz());
  vehicle_state_.set_heading(msg_heading);
  vehicle_state_.set_angular_velocity(
      can_data.adptrin_out_brakesys().da_in_yr_vehdynyawrate_sg() * 3.1416 /
      180.0);

  const double v =
      can_data.adptrin_out_brakesys().da_in_v_vehspdavgdrvn_sg() / 3.6;
  vehicle_state_.set_linear_velocity(v);
  vehicle_state_.set_roll(localization.local_pose().roll());
  vehicle_state_.set_pitch(localization.local_pose().pitch());
  vehicle_state_.set_yaw(localization.local_pose().yaw());

  const double body_ax =
      can_data.adptrin_out_brakesys().da_in_a_vselongtacc_sg();
  vehicle_state_.set_linear_acceleration(body_ax);
  static constexpr double kEpsilon = 1e-2;
  if (std::abs(vehicle_state_.linear_velocity()) < kEpsilon) {
    vehicle_state_.set_kappa(0.0);
  } else {
    vehicle_state_.set_kappa(vehicle_state_.angular_velocity() /
                             vehicle_state_.linear_velocity());
  }
  return true;
}

double VehicleStateProvider::x() const { return vehicle_state_.x(); }

double VehicleStateProvider::y() const { return vehicle_state_.y(); }

double VehicleStateProvider::z() const { return vehicle_state_.z(); }

double VehicleStateProvider::roll() const { return vehicle_state_.roll(); }

double VehicleStateProvider::pitch() const { return vehicle_state_.pitch(); }

double VehicleStateProvider::yaw() const { return vehicle_state_.yaw(); }

double VehicleStateProvider::heading() const {
  return vehicle_state_.heading();
}

double VehicleStateProvider::kappa() const { return vehicle_state_.kappa(); }

double VehicleStateProvider::linear_velocity() const {
  return vehicle_state_.linear_velocity();
}

double VehicleStateProvider::angular_velocity() const {
  return vehicle_state_.angular_velocity();
}

double VehicleStateProvider::linear_acceleration() const {
  return vehicle_state_.linear_acceleration();
}

common::VehicleState::GearPosition VehicleStateProvider::gear() const {
  return vehicle_state_.gear();
}

double VehicleStateProvider::steering_percentage() const {
  return vehicle_state_.steering_percentage();
}

double VehicleStateProvider::timestamp() const {
  return vehicle_state_.timestamp();
}

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.set_linear_velocity(linear_velocity);
}

const VehicleState& VehicleStateProvider::vehicle_state() const {
  return vehicle_state_;
}

::math::Vec2d VehicleStateProvider::EstimateFuturePosition(
    const double t) const {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = vehicle_state_.linear_velocity();
  // Predict distance travel vector
  if (std::fabs(vehicle_state_.angular_velocity()) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state_.angular_velocity() *
                      (1.0 - std::cos(vehicle_state_.angular_velocity() * t));
    vec_distance[1] = std::sin(vehicle_state_.angular_velocity() * t) * v /
                      vehicle_state_.angular_velocity();
  }

  return ::math::Vec2d(vec_distance[0] + vehicle_state_.x(),
                       vec_distance[1] + vehicle_state_.y());
}

::math::Vec2d VehicleStateProvider::ComputeCOMPosition(
    const double rear_to_com_distance) const {
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v;
  Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                          vehicle_state_.z());
  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;

  return ::math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}
double VehicleStateProvider::getYawFromQuaternion(double w, double x, double y,
                                                  double z) {
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}

}  // namespace common
}  // namespace planning
}  // namespace zark
