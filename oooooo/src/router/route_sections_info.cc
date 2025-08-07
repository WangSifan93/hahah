#include "router/route_sections_info.h"

#include <algorithm>
#include <utility>

namespace e2e_noa::planning {
namespace {
template <typename U>
std::vector<mapping::ElementId> ConvertFrom(
    const google::protobuf::RepeatedPtrField<U>& fields) {
  std::vector<mapping::ElementId> c;
  c.reserve(fields.size());
  for (const auto& field : fields) {
    c.push_back(mapping::ElementId(field));
  }
  return c;
}
}  // namespace

RouteSectionsInfo::RouteSectionsInfo(const PlannerSemanticMapManager& psmm,
                                     const RouteSections* sections) {
  route_sections_ = CHECK_NOTNULL(sections);
  if (sections->empty()) return;

  section_segments_.reserve(route_sections_->size());
  int i = 0;
  for (; i < route_sections_->size(); ++i) {
    const auto& section_info =
        psmm.FindSectionByIdOrNull(route_sections_->section_ids()[i]);
    if (section_info == nullptr) {
      section_segments_.emplace_back(RouteSectionSegmentInfo{
          .id = route_sections_->section_ids()[i],
          .start_fraction = 0.0,
          .end_fraction = 1.0,
          .average_length = 0.0,
          .lane_ids = std::vector<mapping::ElementId>()});
      continue;
    }
    section_segments_.emplace_back(
        RouteSectionSegmentInfo{.id = route_sections_->section_ids()[i],
                                .start_fraction = 0.0,
                                .end_fraction = 1.0,
                                .average_length = section_info->curve_length(),
                                .lane_ids = section_info->lanes()});
  }

  section_segments_.front().start_fraction = route_sections_->start_fraction();
  section_segments_.back().end_fraction = route_sections_->end_fraction();

  for (auto& seg : section_segments_) {
    absl::flat_hash_map<mapping::ElementId, int> id_idx_map;
    for (int i = 0; i < seg.lane_ids.size(); ++i) {
      id_idx_map.insert({seg.lane_ids[i], i});
    }
    seg.id_idx_map = std::move(id_idx_map);
  }
  planning_horizon_ = sections->planning_horizon(psmm);
}

RouteSectionsInfo::RouteSectionsInfo(const ad_e2e::planning::Map& v2smm,
                                     const RouteSections* sections) {
  route_sections_ = CHECK_NOTNULL(sections);
  if (sections->empty()) return;

  section_segments_.reserve(route_sections_->size());
  int i = 0;
  for (; i < route_sections_->size(); ++i) {
    const auto& section =
        v2smm.GetSectionById(route_sections_->section_ids()[i]).get();
    if (section == nullptr) {
      section_segments_.emplace_back(RouteSectionSegmentInfo{
          .id = route_sections_->section_ids()[i],
          .start_fraction = 0.0,
          .end_fraction = 1.0,
          .average_length = 0.0,
          .lane_ids = std::vector<mapping::ElementId>()});
      continue;
    }
    std::vector<mapping::ElementId> lane_ids = section->lanes();
    section_segments_.emplace_back(
        RouteSectionSegmentInfo{.id = route_sections_->section_ids()[i],
                                .start_fraction = 0.0,
                                .end_fraction = 1.0,
                                .average_length = section->curve_length(),
                                .lane_ids = std::move(lane_ids)});
  }

  section_segments_.front().start_fraction = route_sections_->start_fraction();
  section_segments_.back().end_fraction = route_sections_->end_fraction();

  for (auto& seg : section_segments_) {
    absl::flat_hash_map<mapping::ElementId, int> id_idx_map;
    for (size_t i = 0; i < seg.lane_ids.size(); ++i) {
      id_idx_map.insert({seg.lane_ids[i], i});
    }
    seg.id_idx_map = std::move(id_idx_map);
  }
  planning_horizon_ = sections->planning_horizon(v2smm);
}
}  // namespace e2e_noa::planning
