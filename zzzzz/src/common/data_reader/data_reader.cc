/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file data_reader.cc
 **/

#include "apps/planning/src/common/data_reader/data_reader.h"

namespace zark {
namespace planning {

std::shared_ptr<DataReader> DataReader::data_reader_ptr_ = nullptr;

std::shared_ptr<DataReader> DataReader::GetInstance() {
  static std::mutex data_reader_mutex_;
  if (data_reader_ptr_ == nullptr) {
    std::lock_guard<std::mutex> lock(data_reader_mutex_);
    // double check;
    if (data_reader_ptr_ == nullptr) {
      data_reader_ptr_.reset(new DataReader());
    }
  }
  return data_reader_ptr_;
}

void DataReader::SetMapData(const MapInfo& msg) {
  std::lock_guard<std::mutex> lock(map_lock_);
  local_map_ = msg;
}

void DataReader::SetLocalizationData(
    const zark::localization::LocalizationInfo& msg) {
  std::lock_guard<std::mutex> lock(localization_lock_);
  localization_estimate_ = msg;
}

void DataReader::SetMapState(const zark::mapfusion::MapLocalizationInfo& msg) {
  std::lock_guard<std::mutex> lock(map_state_lock_);
  map_localization_ = msg;
}

void DataReader::SetFittedResult(
    const std::pair<uint64_t, zark::ads_decision::DEC_Outputs>& msg) {
  std::lock_guard<std::mutex> lock(fitted_lines_lock_);
  fitted_lines_ = msg;
}

void DataReader::SetForceLCResult(const ads_decision::DEC_Outputs& msg) {
  std::lock_guard<std::mutex> lock(force_lc_lock_);
  force_lc_result_ = msg;
}

void DataReader::SetGapChoiceResult(const GapChoiceInfo& msg) {
  std::lock_guard<std::mutex> lock(force_lc_lock_);
  auto tunnel_info = force_lc_result_.mutable_dec_out_planpropinfo_bus();
  tunnel_info->set_dec_out_is_lanechggapexist_bl(msg.has_gap);
  tunnel_info->set_dec_out_lanechghazobjid1_u32(msg.lc_front_hazard_id);
  tunnel_info->set_dec_out_lanechghazobjid2_u32(msg.lc_back_hazard_id);
}

void DataReader::SetNaviResult(const planning::NavigationInfo& msg) {
  std::lock_guard<std::mutex> lock(navi_info_lock_);
  navi_lane_change_info_ = msg;
}

void DataReader::SetMeasureInfo(const ForceLcMeasureInfo& msg) {
  std::lock_guard<std::mutex> lock(measure_info_lock_);
  dec_measure_info_ = msg;
}

void DataReader::SetRefLineState(
    const reference_line_proto::ReferenceLineState& msg) {
  std::lock_guard<std::mutex> lock(refline_sate_lock_);
  refline_state_ = msg;
}

const MapInfo DataReader::GetMapData() {
  std::lock_guard<std::mutex> lock(map_lock_);
  return local_map_;
}

zark::localization::LocalizationInfo DataReader::GetLocalizationData() {
  std::lock_guard<std::mutex> lock(localization_lock_);
  return localization_estimate_;
}

const zark::mapfusion::MapLocalizationInfo DataReader::GetMapState() {
  std::lock_guard<std::mutex> lock(map_state_lock_);
  return map_localization_;
}

void DataReader::ClearMapData(const MapInfo& msg) {
  std::lock_guard<std::mutex> lock(map_lock_);
  local_map_.Reset();
}

const std::pair<uint64_t, zark::ads_decision::DEC_Outputs>
DataReader::GetFittedResult() {
  std::lock_guard<std::mutex> lock(fitted_lines_lock_);
  return fitted_lines_;
}

const ads_decision::DEC_Outputs DataReader::GetForceLCResult() {
  std::lock_guard<std::mutex> lock(force_lc_lock_);
  return force_lc_result_;
}

const NavigationInfo DataReader::GetNaviResult() {
  std::lock_guard<std::mutex> lock(navi_info_lock_);
  return navi_lane_change_info_;
}

const ForceLcMeasureInfo DataReader::GetMeasureInfo() {
  std::lock_guard<std::mutex> lock(measure_info_lock_);
  return dec_measure_info_;
}

const reference_line_proto::ReferenceLineState DataReader::GetRefLineState() {
  std::lock_guard<std::mutex> lock(refline_sate_lock_);
  return refline_state_;
}

}  // namespace planning
}  // namespace zark
