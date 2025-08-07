#ifndef ONBOARD_MAPS_SEMANTIC_MAP_DEFS_H_
#define ONBOARD_MAPS_SEMANTIC_MAP_DEFS_H_

#include <cstdint>
#include <limits>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "container/strong_int.h"
#include "maps/type_defs.h"

namespace e2e_noa {
namespace mapping {

DEFINE_STRONG_INT_TYPE(SegmentId, int64_t);
using ElementId = std::string;
using SectionId = std::string;

const ElementId kInvalidElementId("");
const SectionId kInvalidSectionId("");

using SectionConnection = std::pair<SectionId, SectionId>;
using SectionConnectionDistance =
    absl::flat_hash_map<SectionConnection, double>;
namespace v2 {
struct Segment {
  ElementId element_id;
  SegmentId segment_id;
};
}  // namespace v2
}  // namespace mapping
}  // namespace e2e_noa

#endif
