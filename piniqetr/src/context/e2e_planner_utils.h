#ifndef E2E_PLANNER_UTILS_H_
#define E2E_PLANNER_UTILS_H_

#include <memory>
#include <vector>

#include "math/vec.h"
#include "scene/construction_scene_identification.h"

namespace e2e_noa::planning {
bool GetInterestPointsOfLatSafetyCheck(const Vec2d& ego_pos,
                                       const double& ego_v,
                                       const LaneSequencePtr& tgt_seq,
                                       std::vector<double>* ipts);
}
#endif