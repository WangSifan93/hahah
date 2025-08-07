#include "common/object_situation.h"
namespace ad_e2e::planning {
void ObjectSituation::Reset() {
  lon_distance_ = 0.0;
  lat_distance_ = 0.0;
  spacetime_ = {};
}

}  // namespace ad_e2e::planning
