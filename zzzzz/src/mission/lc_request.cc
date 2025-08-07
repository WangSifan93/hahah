/******************************************************************************
 * Copyright 2024 The zark Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lc_request.cc
 **/

#include "apps/planning/src/mission/lc_request.h"

namespace zark {
namespace planning {

constexpr double kFuryValueUpperBound = 100.0;

LCRequest::LCRequest(const MissionDeciderConfig& config) {
  config_ = config;
  mission_.Reset();
  mission_.lc_flags.Reset();
  default_urgency_ =
      0.5 * (config_.lc_request.urgency_max + config_.lc_request.urgency_min);
}

LCRequest::~LCRequest() {}

Mission LCRequest::Process(Frame* frame) {
  // TODO: efficient lc process;
  // TODO: lane change state machine running;

  // set lc request and type from fct info or inner state machine
  const auto fct_out_bus_lineinfo =
      frame->local_view().fct_output->fct_out_bus_lineinfo();
  const auto fct_lc_status = fct_out_bus_lineinfo.arb_sts_arbalcstate_ealcs();
  const auto fct_lc_reason = fct_out_bus_lineinfo.arb_sts_arblnechgrsn_elcr();

  // check need lc
  bool need_lc = false;
  bool use_left_refline = false;
  bool use_right_refline = false;
  bool crossed_line = CheckCrossedLine(
      fct_lc_status, *frame->FindCurrentLocalRoute(), arriving_target_line_);
  if (fct_lc_status == ads_common::eALCS_ALCState::ALCS_LeftLaneChange ||
      fct_lc_status == ads_common::eALCS_ALCState::ALCS_LftLaneChangeWait ||
      fct_lc_status == ads_common::eALCS_ALCState::ALCS_LeftForceLaneChange) {
    mission_.lc_request = Mission::LaneChangeRequest::LC_LEFT;
    need_lc = true;
    if (!crossed_line) {
      use_left_refline = true;
      mission_.target_reference_line =
          Mission::TargetReflineType::REF_LEFT_LANE;
    } else {
      mission_.target_reference_line =
          Mission::TargetReflineType::REF_CURRENT_LANE;
    }
  }
  if (fct_lc_status == ads_common::eALCS_ALCState::ALCS_RightLaneChange ||
      fct_lc_status == ads_common::eALCS_ALCState::ALCS_RghtLaneChangeWait ||
      fct_lc_status == ads_common::eALCS_ALCState::ALCS_RightForceLaneChange) {
    mission_.lc_request = Mission::LaneChangeRequest::LC_RIGHT;
    need_lc = true;
    if (!crossed_line) {
      use_right_refline = true;
      mission_.target_reference_line =
          Mission::TargetReflineType::REF_RIGHT_LANE;
    } else {
      mission_.target_reference_line =
          Mission::TargetReflineType::REF_CURRENT_LANE;
    }
  }

  // check lc type;
  if (need_lc) {
    switch (fct_lc_reason) {
      case zark::ads_common::eLCR_LaneChangeReason::LCR_RoutTskLneChng:
        mission_.lc_type = Mission::LaneChangeType::ROUTING_LC;
        // TODO: calculate routing lc urgency
        mission_.lc_urgency = default_urgency_;
        break;
      case zark::ads_common::eLCR_LaneChangeReason::LCR_DrvrLneChng:
        mission_.lc_type = Mission::LaneChangeType::MANUAL_LC;
        // TODO: calculate manual lc urgency
        mission_.lc_urgency = default_urgency_;
        break;
      case zark::ads_common::eLCR_LaneChangeReason::LCR_EffLneChng:
        mission_.lc_type = Mission::LaneChangeType::EFFICIENT_LC;
        // TODO: calculate efficient lc urgency
        mission_.lc_urgency = default_urgency_;
        break;
      default:
        break;
    }
    mission_.lc_flags.is_need_lc = need_lc;
    if (fct_lc_status == ads_common::eALCS_ALCState::ALCS_LeftLaneChange ||
        fct_lc_status == ads_common::eALCS_ALCState::ALCS_RightLaneChange ||
        fct_lc_status == ads_common::eALCS_ALCState::ALCS_LeftForceLaneChange ||
        fct_lc_status ==
            ads_common::eALCS_ALCState::ALCS_RightForceLaneChange) {
      mission_.is_lc_ready = true;
    }

  } else {
    mission_.Reset();
    mission_.lc_flags.Reset();
  }
  // update near ramp info
  mission_.is_near_ramp = frame->GetNoaDecisionMeas().ego_info_.is_near_ramp_;
  UpdateTargetLocalRoute(use_left_refline, use_right_refline,
                         *frame->FindCurrentLocalRoute());
  return mission_;
}

bool LCRequest::CheckCrossedLine(const ads_common::eALCS_ALCState& fct_lc_state,
                                 const LocalRoute& current_local_route,
                                 bool& arriving_target_line) {
  bool res = false;
  if (fct_lc_state == ads_common::eALCS_ALCState::ALCS_LeftLaneChange ||
      fct_lc_state == ads_common::eALCS_ALCState::ALCS_RightLaneChange ||
      fct_lc_state == ads_common::eALCS_ALCState::ALCS_LftLaneChangeWait ||
      fct_lc_state == ads_common::eALCS_ALCState::ALCS_RghtLaneChangeWait ||
      fct_lc_state == ads_common::eALCS_ALCState::ALCS_LeftForceLaneChange ||
      fct_lc_state == ads_common::eALCS_ALCState::ALCS_RightForceLaneChange) {
    if (arriving_target_line) {
      res = true;
    } else {
      if (fct_lc_state == ads_common::eALCS_ALCState::ALCS_LeftLaneChange ||
          fct_lc_state == ads_common::eALCS_ALCState::ALCS_LftLaneChangeWait ||
          fct_lc_state ==
              ads_common::eALCS_ALCState::ALCS_LeftForceLaneChange) {
        if (current_local_route.IsHostLC2Left() ||
            fct_lc_state ==
                ads_common::eALCS_ALCState::ALCS_LeftForceLaneChange) {
          AINFO << "cross left line, finish current lane "
                   "change.";
          arriving_target_line = true;
          res = true;
        }
      } else if (fct_lc_state ==
                     ads_common::eALCS_ALCState::ALCS_RightLaneChange ||
                 fct_lc_state ==
                     ads_common::eALCS_ALCState::ALCS_RghtLaneChangeWait ||
                 fct_lc_state ==
                     ads_common::eALCS_ALCState::ALCS_RightForceLaneChange) {
        if (current_local_route.IsHostLC2Right() ||
            fct_lc_state ==
                ads_common::eALCS_ALCState::ALCS_RightForceLaneChange) {
          AINFO << "cross right line, finish current lane "
                   "change.";
          arriving_target_line = true;
          res = true;
        }
      }
    }
  } else {
    arriving_target_line = false;
    res = false;
  }
  return res;
}

void LCRequest::UpdateTargetLocalRoute(const bool use_left_refline,
                                       const bool use_right_refline,
                                       LocalRoute& current_local_route) {
  current_local_route.SetIsTargetRoute(true);
  if (use_left_refline) {
    current_local_route.SetIsTargetRoute(false);
    if (current_local_route.LeftRoute()) {
      current_local_route.LeftRoute()->SetIsTargetRoute(true);
    } else {
      AWARN << "need lc to left, but find left local route error.";
    }
    if (current_local_route.RightRoute()) {
      current_local_route.RightRoute()->SetIsTargetRoute(false);
    }
  } else if (use_right_refline) {
    current_local_route.SetIsTargetRoute(false);
    if (current_local_route.LeftRoute()) {
      current_local_route.LeftRoute()->SetIsTargetRoute(false);
    }
    if (current_local_route.RightRoute()) {
      current_local_route.RightRoute()->SetIsTargetRoute(true);
    } else {
      AWARN << "need lc to right, but find right local route error.";
    }
  }
}

bool LCRequest::GetIsNearRamp() { return mission_.is_near_ramp; }

}  // namespace planning
}  // namespace zark
