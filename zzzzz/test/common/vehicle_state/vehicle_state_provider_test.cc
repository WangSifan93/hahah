/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <string>

#include "Eigen/Core"
#include "gtest/gtest.h"

using namespace zark::planning::common;
using zark::localization::LocalizationInfo;

class VehicleStateProviderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    zark::localization::OdometryInfo *local_pose =
        localization_.mutable_local_pose();
    zark::localization::Pose6DOF *pose = local_pose->mutable_pose();
    zark::localization::Point3D *velocity = local_pose->mutable_velocity();
    zark::localization::Point3D *acceleration =
        local_pose->mutable_acceleration();
    zark::localization::Point3D *t = pose->mutable_t();
    zark::localization::Quaterniond *q = pose->mutable_q();
    t->set_x(357.51331791372041);
    t->set_y(96.165912376788725);
    t->set_z(-31.983237908221781);
    q->set_qx(0.024015498296453403);
    q->set_qy(0.0021656820647661572);
    q->set_qz(-0.99072964388722151);
    q->set_qw(-0.13369120534226134);
    velocity->set_x(-1.5626866011382312);
    velocity->set_y(-5.9852188341040344);
    velocity->set_z(0.024798037277423912);
    acceleration->set_x(-0.68775567663756187);
    acceleration->set_y(-0.079383290718229638);
    acceleration->set_z(-0.43889982872693695);
    local_pose->set_yawrate(0.0035469168217162248);
    local_pose->set_roll(0);
    local_pose->set_pitch(0);
    local_pose->set_yaw(0);
    chassis_.mutable_adptrin_out_brakesys()->set_da_in_v_vehspdavgdrvn_sg(18.0);
    chassis_.mutable_adptrin_out_brakesys()->set_da_in_yr_vehdynyawrate_sg(0.0);
    chassis_.mutable_adptrin_out_brakesys()->set_da_in_a_vselongtacc_sg(0.0);

    // chassis_.set_speed_mps(3.0);
    // chassis_.set_gear_location(Chassis::GearPosition::GEAR_DRIVE);
  }

 protected:
  LocalizationInfo localization_;
  zark::ads_adptrin::ADAS_Inputs chassis_;
};

TEST_F(VehicleStateProviderTest, Accessors) {
  auto vehicle_state_provider = std::make_shared<VehicleStateProvider>();
  vehicle_state_provider->Update(localization_, chassis_);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->x(), 357.51331791372041);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->y(), 96.165912376788725);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->heading(), 2.8730765882060623);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->roll(), 0.0);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->pitch(), 0.0);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->yaw(), 0.0);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->linear_velocity(), 5.0);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->angular_velocity(), 0.0);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->linear_acceleration(), 0.0);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->gear(), 0);
}

TEST_F(VehicleStateProviderTest, EstimateFuturePosition) {
  auto vehicle_state_provider = std::make_shared<VehicleStateProvider>();
  vehicle_state_provider->Update(localization_, chassis_);
  math::Vec2d future_position =
      vehicle_state_provider->EstimateFuturePosition(1.0);
  EXPECT_NEAR(future_position.x(), 357.5133179, 1e-3);
  EXPECT_NEAR(future_position.y(), 101.165912, 1e-3);
  future_position = vehicle_state_provider->EstimateFuturePosition(2.0);
  EXPECT_NEAR(future_position.x(), 357.5133179, 1e-3);
  EXPECT_NEAR(future_position.y(), 106.165912, 1e-3);
}
