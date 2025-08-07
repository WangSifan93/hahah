#include "context/e2e_planner_utils.h"
namespace e2e_noa::planning {

bool GetInterestPointsOfLatSafetyCheck(const Vec2d& ego_pos,
                                       const double& ego_v,
                                       const LaneSequencePtr& tgt_seq,
                                       std::vector<double>* ipts) {
  if (!tgt_seq || !ipts) return false;

  double ego_s = 0.0;
  tgt_seq->GetProjectionDistance(ego_pos, &ego_s);

  std::vector<double> check_time{0.0, 1.0, 2.0, 3.0};
  for (const auto& t : check_time) {
    auto pt_0 = tgt_seq->GetPointAtS(ego_s + ego_v * t);
    auto pt_1 = tgt_seq->GetPointAtS(ego_s + ego_v * t + 1.0);
    if (pt_0 == pt_1 || ego_s + ego_v * t + 1.0 > tgt_seq->GetPointsLength()) {
      break;
    }
    ipts->emplace_back(
        ad_e2e::planning::math::NormalizeAngle((pt_1 - pt_0).Angle()));

    std::vector<ad_e2e::planning::Point2d> debug;
    debug.emplace_back(pt_0);
    debug.emplace_back(pt_1);
  }
  return true;
}

}  // namespace e2e_noa::planning