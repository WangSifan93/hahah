#ifndef MAPS_SMM_PROTO_UTIL_H_
#define MAPS_SMM_PROTO_UTIL_H_

#include <memory>

#include "maps/map.h"
#include "maps/semantic_map_defs.h"

namespace e2e_noa::mapping {

template <typename SmmType>
ad_e2e::planning::LaneConstPtr FindLanePtr(const SmmType& smm,
                                           mapping::ElementId id) {
  if constexpr (std::is_same_v<SmmType, ad_e2e::planning::Map>) {
    return smm.GetLaneById(id);
  } else {
    return smm.FindLaneByIdOrNull(id);
  }
}

template <typename SmmType>
ad_e2e::planning::SectionConstPtr FindSectionPtr(const SmmType& smm,
                                                 mapping::SectionId id) {
  if constexpr (std::is_same_v<SmmType, ad_e2e::planning::Map>) {
    return smm.GetSectionById(id);
  } else {
    return smm.FindSectionByIdOrNull(id);
  }
}

}  

#endif
