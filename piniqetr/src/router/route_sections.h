#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_H_

#include <stddef.h>

#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "maps/lane_path.h"
#include "maps/lane_point.h"
#include "maps/semantic_map_defs.h"
#include "plan/planner_semantic_map_manager.h"
#include "route.pb.h"

namespace e2e_noa::planning {

class RouteSections {
 public:
  struct RouteSectionSegment {
    mapping::SectionId id;
    double start_fraction;
    double end_fraction;
  };

  RouteSections() {}
  explicit RouteSections(double start_fraction, double end_fraction,
                         std::vector<mapping::SectionId> section_ids,
                         mapping::LanePoint destination)
      : start_fraction_(start_fraction),
        end_fraction_(end_fraction),
        section_ids_(std::move(section_ids)),
        destination_(std::move(destination)) {}

  static RouteSections BuildFromLanePath(const PlannerSemanticMapManager& psmm,
                                         const mapping::LanePath& lane_path);

  static RouteSections BuildFromProto(const RouteSectionSequenceProto& proto);

  bool empty() const { return section_ids_.empty(); }
  int size() const { return section_ids_.size(); }
  int section_id_size() const { return section_ids_.size(); }

  bool operator==(const RouteSections& other) const {
    if (start_fraction_ != other.start_fraction_) return false;
    if (end_fraction_ != other.end_fraction_) return false;
    if (size() != other.size()) return false;
    if (destination_ != other.destination()) return false;

    for (size_t i = 0; i < section_ids_.size(); ++i) {
      if (section_ids_[i] != other.section_id(i)) return false;
    }
    return true;
  }
  bool operator!=(const RouteSections& other) const {
    return !(*this == other);
  }

  double start_fraction() const { return start_fraction_; }
  double end_fraction() const { return end_fraction_; }
  void set_topology_start_offset(double topology_start_offset) {
    topology_start_offset_ = topology_start_offset;
  }
  double topology_start_offset() const { return topology_start_offset_; }
  absl::Span<const mapping::SectionId> section_ids() const {
    return section_ids_;
  }
  mapping::SectionId section_id(size_t index) const {
    CHECK_LT(index, section_ids_.size());
    return section_ids_[index];
  }

  RouteSectionSegment route_section_segment(int index) const;
  RouteSectionSegment front() const {
    CHECK_GE(section_ids_.size(), 1) << "Route sections empty!";
    return route_section_segment(0);
  }
  RouteSectionSegment back() const {
    CHECK_GE(section_ids_.size(), 1) << "Route sections empty!";
    return route_section_segment(section_ids_.size() - 1);
  }

  mapping::LanePoint destination() const { return destination_; }

  absl::StatusOr<int> FindSectionSegment(
      const RouteSectionSegment& segment) const;

  void Clear() { *this = RouteSections(); }
  std::string DebugString() const {
    return absl::StrCat("section ids:", absl::StrJoin(section_ids_, ","),
                        ", start fraction:", start_fraction_,
                        ", end_fraction:", end_fraction_);
  }

  double planning_horizon(const PlannerSemanticMapManager& psmm) const;
  double planning_horizon(const ad_e2e::planning::Map& v2smm) const;
  void ToProto(RouteSectionSequenceProto* proto) const;

 private:
  double topology_start_offset_ = 0.0;
  double start_fraction_{0};
  double end_fraction_{0};
  std::vector<mapping::SectionId> section_ids_;

  mapping::LanePoint destination_;

  mutable std::optional<double> planning_horizon_ = std::nullopt;
};

}  // namespace e2e_noa::planning

#endif
