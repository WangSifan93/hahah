#include "speed/st_boundary_with_decision.h"

#include <cmath>
#include <ostream>

namespace e2e_noa::planning {

StBoundaryWithDecision::StBoundaryWithDecision(StBoundaryRef raw_st_boundary)
    : raw_st_boundary_(std::move(raw_st_boundary)),
      st_boundary_(StBoundary::CopyInstance(*raw_st_boundary_)) {}

StBoundaryWithDecision::StBoundaryWithDecision(
    StBoundaryRef raw_st_boundary, StBoundaryProto::DecisionType decision_type,
    StBoundaryProto::DecisionReason decision_reason)
    : raw_st_boundary_(std::move(raw_st_boundary)),
      st_boundary_(StBoundary::CopyInstance(*raw_st_boundary_)),
      decision_type_(decision_type),
      decision_reason_(decision_reason) {}

StBoundaryWithDecision::StBoundaryWithDecision(
    StBoundaryRef raw_st_boundary, StBoundaryProto::DecisionType decision_type,
    StBoundaryProto::DecisionReason decision_reason, std::string decision_info,
    double follow_standstill_distance, double lead_standstill_distance,
    double pass_time, double yield_time)
    : raw_st_boundary_(std::move(raw_st_boundary)),
      st_boundary_(StBoundary::CopyInstance(*raw_st_boundary_)),
      decision_type_(decision_type),
      decision_reason_(decision_reason),
      decision_info_(std::move(decision_info)),
      follow_standstill_distance_(follow_standstill_distance),
      lead_standstill_distance_(lead_standstill_distance) {
  SetTimeBuffer(pass_time, yield_time);
}

void StBoundaryWithDecision::SetTimeBuffer(double pass_time,
                                           double yield_time) {
  st_boundary_ = StBoundary::CopyInstance(*raw_st_boundary_);
  pass_time_ = pass_time;
  yield_time_ = yield_time;
  st_boundary_->ExpandByT(pass_time_, yield_time_);
}

}  // namespace e2e_noa::planning
