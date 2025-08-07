#ifndef PLANNER_SPEED_ST_BOUNDARY_WITH_DECISION_H_
#define PLANNER_SPEED_ST_BOUNDARY_WITH_DECISION_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "speed/st_boundary.h"
#include "speed/st_point.h"
#include "speed/vt_point.h"
#include "speed_planning.pb.h"

namespace e2e_noa::planning {

class StBoundaryWithDecision {
 public:
  explicit StBoundaryWithDecision(StBoundaryRef raw_st_boundary);

  StBoundaryWithDecision(StBoundaryRef raw_st_boundary,
                         StBoundaryProto::DecisionType decision_type,
                         StBoundaryProto::DecisionReason decision_reason);

  StBoundaryWithDecision(StBoundaryRef raw_st_boundary,
                         StBoundaryProto::DecisionType decision_type,
                         StBoundaryProto::DecisionReason decision_reason,
                         std::string decision_info,
                         double follow_standstill_distance,
                         double lead_standstill_distance, double pass_time,
                         double yield_time);

  const StBoundary* st_boundary() const { return st_boundary_.get(); }
  void set_st_boundary(StBoundaryRef st_boundary) {
    st_boundary_ = std::move(st_boundary);
  }

  const StBoundary* raw_st_boundary() const { return raw_st_boundary_.get(); }
  StBoundary* mutable_raw_st_boundary() const { return raw_st_boundary_.get(); }

  StBoundaryProto::DecisionType decision_type() const { return decision_type_; }
  void set_decision_type(StBoundaryProto::DecisionType decision_type) {
    decision_type_ = decision_type;
  }

  const std::string& id() const { return st_boundary()->id(); }
  void set_id(const std::string& id) { st_boundary_->set_id(id); }

  const double ds() const { return raw_st_boundary_->obj_sl_info().ds; }
  const double dl() const { return raw_st_boundary_->obj_sl_info().dl; }
  const FrenetPolygon obj_frenet_polygon() const {
    return raw_st_boundary_->obj_sl_info().frenet_polygon;
  }

  const std::vector<NearestSlPoint>& nearest_sl_points() const {
    return raw_st_boundary_->nearest_sl_points();
  }

  const ObjectDecisionParam decision_param() const {
    return raw_st_boundary_->obj_scenario_info().obj_decision_param;
  }

  const std::optional<std::string>& traj_id() const {
    return st_boundary()->traj_id();
  }
  const std::optional<std::string>& object_id() const {
    return st_boundary()->object_id();
  }

  void InitSTPoints(std::vector<std::pair<StPoint, StPoint>> st_point_pairs) {
    st_boundary_->Init(std::move(st_point_pairs));
  }
  void InitSpeedPoints(std::vector<VtPoint> vt_points) {
    st_boundary_->set_speed_points(std::move(vt_points));
  }

  StBoundaryProto::DecisionReason decision_reason() const {
    return decision_reason_;
  }
  void set_decision_reason(StBoundaryProto::DecisionReason decision_reason) {
    decision_reason_ = decision_reason;
  }

  StBoundaryProto::IgnoreReason ignore_reason() const { return ignore_reason_; }
  void set_ignore_reason(StBoundaryProto::IgnoreReason ignore_reason) {
    ignore_reason_ = ignore_reason;
  }

  double follow_standstill_distance() const {
    return follow_standstill_distance_;
  }
  void set_follow_standstill_distance(double follow_standstill_distance) {
    follow_standstill_distance_ = follow_standstill_distance;
  }
  double lead_standstill_distance() const { return lead_standstill_distance_; }
  void set_lead_standstill_distance(double lead_standstill_distance) {
    lead_standstill_distance_ = lead_standstill_distance;
  }

  double pass_time() const { return pass_time_; }
  double yield_time() const { return yield_time_; }
  void SetTimeBuffer(double pass_time, double yield_time);

  absl::string_view decision_info() const { return decision_info_; }
  void set_decision_info(absl::string_view str) {
    decision_info_ = std::string(str);
  }

  void set_decision_prob(double yield, double pass) {
    decision_prob_.set_yield(yield);
    decision_prob_.set_pass(pass);
  }

  StBoundaryProto::DecisionProb decision_prob() const { return decision_prob_; }

  TrajectoryPointWithAcceleration obj_pose_info() const {
    return raw_st_boundary_->obj_pose_info();
  }

 private:
  StBoundaryRef raw_st_boundary_;

  StBoundaryRef st_boundary_;

  StBoundaryProto::DecisionType decision_type_ = StBoundaryProto::UNKNOWN;
  StBoundaryProto::DecisionReason decision_reason_ =
      StBoundaryProto::UNKNOWN_REASON;
  StBoundaryProto::IgnoreReason ignore_reason_ = StBoundaryProto::NONE;
  std::string decision_info_;
  StBoundaryProto::DecisionProb decision_prob_;

  double follow_standstill_distance_ = 0.0;
  double lead_standstill_distance_ = 0.0;

  double pass_time_ = 0.0;
  double yield_time_ = 0.0;
};

}  // namespace e2e_noa::planning

#endif
