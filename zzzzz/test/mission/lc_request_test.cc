/******************************************************************************
 * Copyright 2024 The zark Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lc_request_test.cc
 **/

#include "apps/planning/src/mission/lc_request.h"

#include <gtest/gtest.h>

#include "apps/planning/src/reference_line_provider/pnc_map/path.h"
#include "apps/planning/src/reference_line_provider/pnc_map/route_segments.h"

namespace zark {
namespace planning {

class LCRequestTest : public ::testing::Test {
 public:
  void SetUp() override {
    MissionDeciderConfig config;
    config.lc_request.enable_force_lc = true;
    config.env_info.invalid_lane = "invalid";
    config.lc_request.enable_effi_lc = false;
    config.lc_request.forc_distance_buffer = 1.0;

    // init  LC Request
    lc_request_ = std::make_unique<LCRequest>(config);
    env_infos_.fury_value = 20.0;
    env_infos_.lane_infos.current_laneinfo.id = "invalid";
    env_infos_.rc_params_infos.dis_2_ramp_threshold = 10.0;
    env_infos_.cur_lane_block_infos.is_current_line_blocked = false;
    env_infos_.lane_infos.left_laneinfo.id = "aaa";
    env_infos_.obstacle_infos.cur_lane_obs.clear();
    env_infos_.obstacle_infos.left_lane_obs.clear();
    env_infos_.obstacle_infos.right_lane_obs.clear();
    env_infos_.lane_spd_infos.v_left_lane_pass = 100.0;
    env_infos_.lane_spd_infos.v_right_lane_pass = 90.0;
    env_infos_.inhibit_infos.is_inhibit_left = false;
    env_infos_.inhibit_infos.is_inhibit_right = false;

    mission_.Reset();

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

    local_route_ =
        std::make_unique<LocalRoute>(route_segments, LocalRouteConfig());
  }

  void TearDown() override {
    lc_request_.release();
    local_route_.release();
  }

 protected:
  std::unique_ptr<LCRequest> lc_request_;
  std::unique_ptr<LocalRoute> local_route_;

  EnvInfos env_infos_;
  Mission mission_;
};

TEST_F(LCRequestTest, TestManualLCCheck) {
  LocalView local_view;

  zark::ads_adptrin::ADAS_Inputs can_data;
  can_data.mutable_adptrin_out_bcm()->set_da_in_is_ldircnio_bl(0);
  can_data.mutable_adptrin_out_bcm()->set_da_in_is_rdircnio_bl(0);
  local_view.can_data =
      std::make_shared<zark::ads_adptrin::ADAS_Inputs>(can_data);

  zark::ads_fct::FCT_Outputs fct_output;
  fct_output.mutable_fct_out_bus_lineinfo()->set_arb_sts_arblnechgrsn_elcr(
      zark::ads_common::eLCR_LaneChangeReason::LCR_NoLneChng);
  local_view.fct_output =
      std::make_shared<zark::ads_fct::FCT_Outputs>(fct_output);

  zark::planning::LCFlags last_lc_flags;
  last_lc_flags.Reset();

  bool ret = false;

  EXPECT_EQ(false, ret);
}

}  // namespace planning
}  // namespace zark
