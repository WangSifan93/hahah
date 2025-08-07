/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_mpc_data_mocker_test.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_mpc_data_mocker.h"

#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"
#include "point_factory.h"

namespace zark {
namespace planning {
class LateralMPCDataMockerTest : public ::testing::Test {
 public:
  void SetUp() override {
    std::string config_file =
        "/zark/apps/planning/test/config/json/"
        "lateral_optimizer_config_test.json";
    Config config{config_file};
    std::ifstream jdata(config_file);
    lateral_config_ =
        config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
    lateral_mpc_data_mocker_ = std::unique_ptr<LateralMPCDataMocker>(
        new LateralMPCDataMocker(lateral_config_));
    ref_line_ptr_ = std::unique_ptr<LateralReference>(
        new LateralReference(lateral_config_));
    common::VehicleConfig vehicle_config;
    vehicle_param_.set_width(2.11);
    vehicle_config.set_vehicle_param(vehicle_param_);
    common::VehicleConfigHelper::Init(vehicle_config);
  }

 protected:
  std::shared_ptr<LateralMPCDataMocker> lateral_mpc_data_mocker_;
  std::unique_ptr<LateralReference> ref_line_ptr_;
  LateralOptimizerConfig lateral_config_;
  common::VehicleParam vehicle_param_;
};

::common::TrajectoryPoint InitializePlanningStartPoint() {
  ::common::PathPoint start_point(1.0, 2.0, 3.0, M_PI_2, 5.0, 6.0, 7.0, 8.0,
                                  "1", 1.0, 2.0);
  GaussianInfo gaussian_info;
  ::common::TrajectoryPoint planning_start_point(start_point, 2.0, 2.5, 3.0,
                                                 0.0, 1.0, gaussian_info);
  planning_start_point.set_omega(0.1);
  return planning_start_point;
}

TEST_F(LateralMPCDataMockerTest, TestPublishCoarseTrajectory) {
  LatMPCData lat_mpc_data;
  const double speed = 1.0;
  const int n_steps = this->lateral_config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  const double dt = this->lateral_config_.model.dt;
  LonMPCData lon_mpc_data;
  std::vector<CorridorPoint> corridor_points;
  lon_mpc_data.t.resize(n_nodes);
  lon_mpc_data.x.resize(1, n_nodes);
  for (int i = 0; i < n_nodes; i++) {
    lon_mpc_data.t[i] = dt * i;
    lon_mpc_data.x(0, i) = speed * lon_mpc_data.t(i);
    CorridorPoint corridor_point;
    corridor_point.s = lon_mpc_data.x(0, i);
    corridor_point.l = 0.0;
    corridor_point.l_left = 1.85;
    corridor_point.l_right = -1.85;
    corridor_point.type_left = CorridorPoint::Type::CURB;
    corridor_point.type_right = CorridorPoint::Type::UNKNOWN;
    corridor_points.push_back(corridor_point);
  }
  LocalRoute local_route;
  Mission mission{};
  CorridorInfo::Type corridor_type;
  Corridor corridor(corridor_points);
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);
  lat_mpc_data = lateral_mpc_data_mocker_->GenerateLateralMPCData(
      corridor_info, lon_mpc_data, ref_line_ptr_.get());

  for (int k = 0; k < n_nodes; k++) {
    EXPECT_DOUBLE_EQ(lat_mpc_data.x_ref(0, k), 0.0);
    EXPECT_DOUBLE_EQ(lat_mpc_data.x_ref(1, k), 0.0);
    EXPECT_DOUBLE_EQ(lat_mpc_data.x_ref(2, k), 0.0);
    EXPECT_DOUBLE_EQ(lat_mpc_data.x_ref(3, k), 0.0);
  }

  for (int k = 0; k < n_steps; k++) {
    EXPECT_DOUBLE_EQ(lat_mpc_data.u_ref(0, k), 0.0);
  }
}

TEST_F(LateralMPCDataMockerTest, TestGenerateEmptyTube) {
  const double speed = 1.0;
  const int n_steps = this->lateral_config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  const double dt = this->lateral_config_.model.dt;
  LonMPCData lon_mpc_data;
  std::vector<CorridorPoint> corridor_points;
  lon_mpc_data.t.resize(n_nodes);
  lon_mpc_data.x.resize(1, n_nodes);
  for (int i = 0; i < n_nodes; i++) {
    lon_mpc_data.t[i] = dt * i;
    lon_mpc_data.x(0, i) = speed * lon_mpc_data.t(i);
    CorridorPoint corridor_point;
    corridor_point.s = lon_mpc_data.x(0, i);
    corridor_point.l = 0.0;
    corridor_point.l_left = 1.85;
    corridor_point.l_right = -1.85;
    corridor_point.type_left = CorridorPoint::Type::CURB;
    corridor_point.type_right = CorridorPoint::Type::UNKNOWN;
    corridor_points.push_back(corridor_point);
  }
  LocalRoute local_route;
  Mission mission{};
  CorridorInfo::Type corridor_type;
  Corridor corridor(corridor_points);

  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);

  auto tube =
      lateral_mpc_data_mocker_->GenerateEmptyTube(corridor_info, lon_mpc_data);
  EXPECT_EQ(tube.pts.size(), n_nodes);
  for (int i = 0; i < n_nodes; i++) {
    EXPECT_DOUBLE_EQ(tube.pts[i].s, lon_mpc_data.x(0, i));
    EXPECT_DOUBLE_EQ(tube.pts[i].l_left_ref, 1.055);
    EXPECT_DOUBLE_EQ(tube.pts[i].l_right_ref, -1.055);
    EXPECT_DOUBLE_EQ(tube.pts[i].l_left_soft, 1.85);
    EXPECT_DOUBLE_EQ(tube.pts[i].l_right_soft, -1.85);
    EXPECT_DOUBLE_EQ(tube.pts[i].l_left_stiff, 1.85);
    EXPECT_DOUBLE_EQ(tube.pts[i].l_right_stiff, -1.85);
    EXPECT_DOUBLE_EQ(tube.pts[i].l_left_hard, 1.85);
    EXPECT_DOUBLE_EQ(tube.pts[i].l_right_hard, -1.85);
  }
}

}  // namespace planning
}  // namespace zark
