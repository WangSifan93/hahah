/******************************************************************************
 * Copyright 2024 The zark Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file env_info_builder_test.cc
 **/

#include <gtest/gtest.h>

#include "apps/planning/src/reference_line_provider/pnc_map/path.h"
#include "apps/planning/src/mission/env_info_builder.h"
#include "apps/planning/src/reference_line_provider/pnc_map/route_segments.h"

namespace zark {
namespace planning {

class EnvInfoBuilderTest : public ::testing::Test {
 public:
  void SetUp() override {
    MissionDeciderConfig config;

    config.env_info.epsilon = 1e-10;
    config.env_info.default_spd_limit = 16.7;
    config.env_info.default_lc_finish_time = 13.0;
    config.env_info.default_lc_congestion_coeff = 1.3;

    // init Env Info Builder
    envinfo_builder_ = std::make_unique<EnvInfoBuilder>(config);

    v_set_ = 13.3;
    vehicle_state_.set_linear_velocity(13.1);

    // Set Local Route
    zark::hdmap_new::Lane lane;
    lane.mutable_id()->set_id("aaa");

    double tmp_inc = 0.0;
    uint32_t tmp_offset = 0.0;
    double tmp_curve = 0.0;
    double tmp_heading = 0.0;
    zark::hdmap_new::Point2D tmp_point;
    tmp_point.set_x(0.0);
    tmp_point.set_y(0.0);

    for (int i = 0; i < 3; i++) {
      tmp_inc += 1.0;
      auto tmp_p = lane.mutable_center_line()->add_point();
      tmp_p->set_x(tmp_point.x() + tmp_inc);
      tmp_p->set_y(tmp_point.y());

      lane.mutable_center_line()->add_offset_cm(tmp_offset);
      lane.mutable_center_line()->add_curve(tmp_curve);
      lane.mutable_center_line()->add_heading_degree(tmp_heading);
    }

    zark::map_service::zmap::LaneInfo lane_info(lane);

    zark::map_service::zmap::LaneInfoConstPtr lane_info_ptr(&lane_info);

    hdmap::LaneSegment lane_segment(lane_info_ptr, 0.0, 3.0);

    zark::hdmap_new::Lane lane1;
    lane1.mutable_id()->set_id("bbb");

    for (int i = 0; i < 3; i++) {
      tmp_inc += 1.0;
      auto tmp_p = lane1.mutable_center_line()->add_point();
      tmp_p->set_x(tmp_point.x() + tmp_inc);
      tmp_p->set_y(tmp_point.y());

      lane1.mutable_center_line()->add_offset_cm(tmp_offset);
      lane1.mutable_center_line()->add_curve(tmp_curve);
      lane1.mutable_center_line()->add_heading_degree(tmp_heading);
    }

    zark::map_service::zmap::LaneInfo lane_info1(lane1);

    zark::map_service::zmap::LaneInfoConstPtr lane_info_ptr1(&lane_info1);

    hdmap::LaneSegment lane_segment1(lane_info_ptr1, 0.0, 10.0);

    hdmap::RouteSegments route_segments;
    route_segments.clear();
    route_segments.emplace_back(lane_segment);
    route_segments.emplace_back(lane_segment1);

    route_segments.SetId("cur");

    local_route_ = std::make_unique<LocalRoute>(route_segments, LocalRouteConfig());
  }

  void TearDown() override {
    envinfo_builder_.release();
    local_route_.release();
  }

 protected:
  std::unique_ptr<EnvInfoBuilder> envinfo_builder_;
  std::unique_ptr<LocalRoute> local_route_;

  double v_set_ = 0.0;
  VehicleState vehicle_state_;
};

TEST_F(EnvInfoBuilderTest, TestUpdateVehicleInfo) {
  VehicleInfos vi_test =
      envinfo_builder_->UpdateVehicleInfo(vehicle_state_, v_set_);
  EXPECT_EQ(13.1, vi_test.cur_adc_v);
  EXPECT_EQ(13.3, vi_test.driver_set_v);
}

TEST_F(EnvInfoBuilderTest, TestUpdateLaneInfo) {
  LaneInfos laneinfo_test =
      envinfo_builder_->UpdateLaneInfo(vehicle_state_, *local_route_.get());
  EXPECT_EQ("cur", laneinfo_test.current_laneinfo.id);
  EXPECT_EQ("", laneinfo_test.left_laneinfo.id);
}

TEST_F(EnvInfoBuilderTest, TestUpdateObstaclesInfo) {
  std::vector<const zark::planning::Obstacle *> obs;
  obs.clear();

  LaneInfos lane_info;
  lane_info.current_lane_width = 3.0;

  ObstacleInfos obsinfo_test =
      envinfo_builder_->UpdateObstaclesInfo(obs, lane_info);

  EXPECT_EQ(0.0, obsinfo_test.cur_lane_obs.size());
  EXPECT_EQ(0.0, obsinfo_test.left_lane_obs.size());
  EXPECT_EQ(0.0, obsinfo_test.lleft_lane_obs.size());
  EXPECT_EQ(0.0, obsinfo_test.right_lane_obs.size());
  EXPECT_EQ(0.0, obsinfo_test.rright_lane_obs.size());
}

TEST_F(EnvInfoBuilderTest, TestIsCurrentLaneBlock) {
  ObstacleInfos obs_info;
  obs_info.cur_lane_obs.clear();
  LaneInfos lane_info;

  CurLaneBlockInfos cur_lane_block_info_test =
      envinfo_builder_->IsCurrentLaneBlock(obs_info, lane_info);

  EXPECT_EQ(false, cur_lane_block_info_test.is_current_line_blocked);
  EXPECT_EQ(0.0, cur_lane_block_info_test.dis_2_frontstaticobj);
}

TEST_F(EnvInfoBuilderTest, TestUpdateLaneSpdInfo) {
  LaneInfos lane_info;
  ObstacleInfos obs_info;

  lane_info.current_laneinfo.id = local_route_->Lanes().Id();
  lane_info.current_laneinfo.local_route = *local_route_.get();
  lane_info.current_sl.set_s(0.0);

  obs_info.cur_lane_obs.clear();
  obs_info.left_lane_obs.clear();
  obs_info.right_lane_obs.clear();

  LaneSpdInfos lane_spd_info =
      envinfo_builder_->UpdateLaneSpdInfo(obs_info, lane_info);

  EXPECT_EQ(lane_spd_info.v_cur_lane_limit, 16.7);
}

TEST_F(EnvInfoBuilderTest, TestUpdateFuryVal) {
  LaneSpdInfos lane_spd_info;
  lane_spd_info.v_cur_lane_limit = 27.78;
  lane_spd_info.v_cur_lane_pass = 27.77;

  double fury_val_test = envinfo_builder_->UpdateFuryVal(lane_spd_info);

  EXPECT_EQ(0.0, fury_val_test);
}

TEST_F(EnvInfoBuilderTest, TestUpdateRouteLCParams) {
  LaneInfos lane_info;
  LaneSpdInfos lane_spd_info;

  lane_info.current_laneinfo.id = local_route_->Lanes().Id();
  lane_info.current_laneinfo.local_route = *local_route_.get();

  lane_spd_info.v_cur_lane_limit = 12.0;

  RCParamInfos rc_param_infos =
      envinfo_builder_->UpdateRouteLCParams(lane_info, lane_spd_info);
  EXPECT_EQ(0.0, rc_param_infos.dis_2_ramp_threshold);
}

}  // namespace planning
}  // namespace zark
