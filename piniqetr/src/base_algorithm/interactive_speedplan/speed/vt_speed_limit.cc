#include "speed/vt_speed_limit.h"

#include <memory>

namespace e2e_noa::planning {

void MergeVtSpeedLimit(const VtSpeedLimit& source, VtSpeedLimit* target) {
  if (source.size() != target->size()) return;
  for (int i = 0; i < source.size(); ++i) {
    if (source[i].speed_limit < (*target)[i].speed_limit) {
      (*target)[i] = source[i];
    }
  }
  return;
}

}  // namespace e2e_noa::planning
