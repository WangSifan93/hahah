#ifndef ST_PLANNING_PREDICTION_PREDICTION_UTIL
#define ST_PLANNING_PREDICTION_PREDICTION_UTIL

#include "prediction/prediction_util.h"

#include <algorithm>
#include <vector>

#include "constraint.pb.h"
#include "math/piecewise_linear_function.h"
#include "object/spacetime_object_trajectory.h"
#include "prediction.pb.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction.h"
#include "prediction/prediction_defs.h"
#include "prediction_common.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace prediction {

void InitTraj(PredictedTrajectory* const trajectory, const double curr_acc,
              std::vector<double>* const split_s) {
  const auto& pts = trajectory->points();
  split_s->emplace_back(0.0);
  double s = 0.0;

  for (size_t i = 0; i < pts.size(); i++) {
    if (i < pts.size() - 1) {
      double dis = (pts.at(i + 1).pos() - pts.at(i).pos()).Length();
      s = s + dis;
      split_s->emplace_back(s);
    }
  }
}

bool RefineTrajByAcc(PredictedTrajectory* const trajectory,
                     const double curr_acc, const double acc_ts_sec) {
  const auto& pts = trajectory->points();
  if (trajectory->points().empty()) return false;
  const double cv_ts_sec = std::max(0.0, kPredictionDuration - acc_ts_sec);

  PredictedTrajectory refined_traj;
  refined_traj.set_probability(trajectory->probability());
  refined_traj.set_index(trajectory->index());
  refined_traj.set_type(trajectory->type());
  refined_traj.mutable_points()->emplace_back(std::move(pts.at(0)));
  std::vector<double> split_s;
  constexpr double t = kPredictionTimeStep;
  InitTraj(trajectory, curr_acc, &split_s);
  Vec2d prev_pos = pts.at(0).pos();
  double prev_s = 0.0;
  double prev_theta = pts.at(0).theta();
  double prev_kappa = pts.at(0).kappa();
  double prev_t = 0.0;
  double prev_v = pts.at(0).v();
  double prev_a = curr_acc;
  double ds = 0.0;
  const int acc_point_num =
      static_cast<int>(acc_ts_sec / kPredictionTimeStep) - 1;
  DLOG(INFO) << "refine init state " << prev_s << " " << prev_theta << " "
             << prev_kappa << " " << prev_t << " " << prev_v << " " << prev_a;
  size_t si = 1;
  for (size_t i = 1; i < kPredictionPointNum; ++i) {
    prev_t = i * t;
    double rest_s = split_s.back() - prev_s;
    double tmp_a = prev_a;
    if (i < acc_point_num + 1) {
      tmp_a = std::min(
          prev_a,
          std::max(
              0.0,
              (rest_s - prev_v * (acc_ts_sec - prev_t) - prev_v * cv_ts_sec) /
                  (0.5 * (acc_ts_sec - prev_t) * (acc_ts_sec - prev_t) +
                   (acc_ts_sec - prev_t))));
      if (prev_v + prev_a * t < 0.0) {
        tmp_a = -prev_v / t;
      }
      ds = prev_v * t + 0.5 * tmp_a * t * t;
    } else {
      tmp_a = 0.0;
      ds = prev_v * t;
    }
    if (prev_s + ds > split_s.at(si)) {
      si = std::min(split_s.size() - 1, si + 1);
    }
    prev_a = tmp_a;
    prev_v += prev_a * t;
    prev_s += ds;

    PredictedTrajectoryPoint refined_pt;
    Vec2d new_pos = (pts.at(si).pos() - pts.at(si - 1).pos()) *
                        (prev_s - split_s.at(si - 1)) /
                        std::max(0.1, split_s.at(si) - split_s.at(si - 1)) +
                    pts.at(si - 1).pos();

    refined_pt.set_pos(new_pos);
    refined_pt.set_s(prev_s);
    refined_pt.set_theta(pts.at(si - 1).theta());
    refined_pt.set_kappa(pts.at(si - 1).kappa());
    refined_pt.set_t(pts.at(0).t() + prev_t);
    refined_pt.set_v(prev_v);
    refined_pt.set_a(prev_a);
    refined_traj.mutable_points()->emplace_back(std::move(refined_pt));
  }
  *trajectory = refined_traj;
  return true;
}

void ExtendPredictionTraj(
    const planning::SpacetimeObjectTrajectory& traj,
    planning::ConstraintProto::LeadingObjectProto* leading_obj) {
  const auto& obj_cur_pos = traj.planner_object().pose().pos();

  PredictedTrajectory extend_traj;
  extend_traj.set_probability(traj.trajectory().probability());
  extend_traj.set_index(traj.trajectory().index());
  bool current_state_insert = false;
  for (size_t i = 0; i < traj.states().size() - 1; ++i) {
    Vec2d base_vec = traj.states().at(i + 1).traj_point->pos() -
                     traj.states().at(i).traj_point->pos();
    Vec2d tgt_vec = obj_cur_pos - traj.states().at(i).traj_point->pos();

    const double pj_length = tgt_vec.Dot(base_vec.normalized());
    if (!current_state_insert) {
      if (pj_length < 0) {
        PredictedTrajectoryPoint pt(traj.planner_object().pose());
        extend_traj.mutable_points()->emplace_back(std::move(pt));
        extend_traj.mutable_points()->emplace_back(
            std::move(*(traj.states().at(i).traj_point)));
        current_state_insert = true;
      } else if (pj_length > 0 && pj_length < base_vec.Length()) {
        PredictedTrajectoryPoint pt(traj.planner_object().pose());
        extend_traj.mutable_points()->emplace_back(std::move(pt));
        current_state_insert = true;
      } else {
        continue;
      }
    } else {
      extend_traj.mutable_points()->emplace_back(
          std::move(*(traj.states().at(i).traj_point)));
    }
  }
  extend_traj.mutable_points()->emplace_back(
      std::move(*(traj.states().back().traj_point)));

  const int origin_traj_size = extend_traj.points().size();
  const int extend_pt_num = kPredictionPointNum - origin_traj_size;
  if (extend_pt_num > 0) {
    Vec2d cv_pos_t(0.0, 0.0);
    if (extend_traj.points().size() > 10) {
      Vec2d dpos =
          extend_traj.points().back().pos() -
          extend_traj.points().at(extend_traj.points().size() - 10).pos();
      double dt = extend_traj.points().back().t() -
                  extend_traj.points().at(extend_traj.points().size() - 10).t();
      cv_pos_t = kPredictionTimeStep * dpos / std::max(0.1, dt);

    } else {
      Vec2d dpos = extend_traj.points().back().pos() -
                   extend_traj.points().front().pos();
      double dt =
          extend_traj.points().back().t() - extend_traj.points().front().t();
      cv_pos_t = kPredictionTimeStep * dpos / std::max(0.1, dt);
    }
    double cv_v = cv_pos_t.Length() / kPredictionTimeStep;
    for (int i = 0; i < extend_pt_num; ++i) {
      PredictedTrajectoryPoint traj_pt;
      traj_pt.set_pos(extend_traj.points().back().pos() + cv_pos_t);
      traj_pt.set_s(extend_traj.points().back().s() + cv_pos_t.Length());
      traj_pt.set_theta(std::atan2(cv_pos_t.x(), cv_pos_t.y()));
      traj_pt.set_kappa(0.0);
      traj_pt.set_t(extend_traj.points().back().t() + kPredictionTimeStep);
      traj_pt.set_v(cv_v);
      traj_pt.set_a(0.0);
      extend_traj.mutable_points()->emplace_back(std::move(traj_pt));
    }
    DLOG(INFO) << "extend traj size: " << extend_traj.points().size();
  }

  planning::SpacetimeObjectTrajectory st_obj_traj(
      traj.planner_object(), std::move(extend_traj), traj.traj_index(),
      traj.required_lateral_gap());
  for (const auto& state : st_obj_traj.states()) {
    state.traj_point->ToProto(leading_obj->add_modified_trajectory());
  }
}

}  // namespace prediction
}  // namespace e2e_noa

#endif
