/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file data_reader.h
 **/

#ifndef DATA_READER_H
#define DATA_READER_H

#include "apps/planning/src/common/data_reader/forcelc_measure.h"
#include "apps/planning/src/common/data_reader/road_section_info.h"
#include "messages/localization/localization.pb.h"
#include "messages/map_service/all_map_new.pb.h"
#include "messages/mapfusion/map_fusion.pb.h"
#include "messages/planning/ads_decision.pb.h"
#include "messages/planning/ads_decision_measure.pb.h"
#include "messages/planning/reference_line.pb.h"

namespace zark {
namespace planning {

struct MapInfo {
  MapInfo()
      : map_type(
            hdmap_new::MapHeader::MapType::MapHeader_MapType_MAP_TYPE_NONE),
        map_timestamp(-1),
        map_lane_size(0),
        map_path(),
        guidance_info(),
        navigation_info() {}

  void Reset() {
    map_type = hdmap_new::MapHeader::MapType::MapHeader_MapType_MAP_TYPE_NONE;
    map_timestamp = -1;
    map_path.Clear();
    guidance_info.Clear();
  }

 public:
  hdmap_new::MapHeader::MapType map_type;
  int64_t map_timestamp;
  int map_lane_size;
  google::protobuf::RepeatedPtrField<hdmap_new::Path> map_path;
  hdmap_new::Guidance guidance_info;
  hdmap_new::NavigationInfo navigation_info;
};

struct GapChoiceInfo {
  GapChoiceInfo()
      : has_gap(false), lc_front_hazard_id(0), lc_back_hazard_id(0) {}

  GapChoiceInfo(bool gap, uint32_t front_id, uint32_t back_id)
      : has_gap(gap),
        lc_front_hazard_id(front_id),
        lc_back_hazard_id(back_id) {}

  void Reset() {
    has_gap = false;
    lc_front_hazard_id = 0;
    lc_back_hazard_id = 0;
  }

 public:
  bool has_gap;
  uint32_t lc_front_hazard_id;
  uint32_t lc_back_hazard_id;
};

class DataReader {
 public:
  virtual ~DataReader(){};

  static std::shared_ptr<DataReader> GetInstance();

  void SetMapData(const MapInfo& msg);

  void SetLocalizationData(const zark::localization::LocalizationInfo& msg);

  void SetMapState(const zark::mapfusion::MapLocalizationInfo& msg);

  void SetFittedResult(
      const std::pair<uint64_t, zark::ads_decision::DEC_Outputs>& msg);

  void SetForceLCResult(const ads_decision::DEC_Outputs& msg);

  void SetGapChoiceResult(const GapChoiceInfo& msg);

  void SetNaviResult(const planning::NavigationInfo& msg);

  void SetMeasureInfo(const ForceLcMeasureInfo& msg);

  void SetRefLineState(const reference_line_proto::ReferenceLineState& msg);

  const MapInfo GetMapData();

  zark::localization::LocalizationInfo GetLocalizationData();

  const zark::mapfusion::MapLocalizationInfo GetMapState();

  void ClearMapData(const MapInfo& msg);

  const std::pair<uint64_t, zark::ads_decision::DEC_Outputs> GetFittedResult();

  const ads_decision::DEC_Outputs GetForceLCResult();

  const NavigationInfo GetNaviResult();

  const ForceLcMeasureInfo GetMeasureInfo();

  const reference_line_proto::ReferenceLineState GetRefLineState();

 private:
  DataReader() = default;
  DataReader(const DataReader& dr) = delete;
  DataReader& operator=(const DataReader& dr) = delete;

 private:
  static std::shared_ptr<DataReader> data_reader_ptr_;

  std::mutex map_lock_;
  std::mutex localization_lock_;
  std::mutex map_state_lock_;
  std::mutex fitted_lines_lock_;
  std::mutex force_lc_lock_;
  std::mutex navi_info_lock_;
  std::mutex measure_info_lock_;
  std::mutex refline_sate_lock_;

  // External Input Data for local route provider
  zark::localization::LocalizationInfo localization_estimate_;
  zark::mapfusion::MapLocalizationInfo map_localization_;
  MapInfo local_map_;

  // Internal Data from local route provider and mission
  std::pair<uint64_t, zark::ads_decision::DEC_Outputs> fitted_lines_;
  zark::ads_decision::DEC_Outputs force_lc_result_;
  zark::planning::NavigationInfo navi_lane_change_info_;
  zark::planning::ForceLcMeasureInfo dec_measure_info_;
  reference_line_proto::ReferenceLineState refline_state_;
};

}  // namespace planning
}  // namespace zark
#endif  // end of DATA_READER_H
