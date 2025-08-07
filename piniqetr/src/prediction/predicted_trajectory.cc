#include "prediction/predicted_trajectory.h"

#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "math/geometry/util.h"
#include "prediction/prediction_defs.h"

namespace e2e_noa {
namespace prediction {

void PredictedTrajectoryPoint::FromProto(
    const PredictedTrajectoryPointProto& proto) {
  set_pos(Vec2dFromProto(proto.pos()));
  set_s(proto.s());
  set_theta(proto.theta());
  set_kappa(proto.kappa());

  set_t(proto.t());
  set_v(proto.v());
  set_a(proto.a());
}

void PredictedTrajectoryPoint::ToProto(
    PredictedTrajectoryPointProto* proto) const {
  Vec2dToProto(pos(), proto->mutable_pos());
  proto->set_s(s());
  proto->set_theta(theta());
  proto->set_kappa(kappa());
  proto->set_t(t());
  proto->set_v(v());
  proto->set_a(a());
}

std::string PredictedTrajectory::DebugString() const {
  return absl::StrFormat("type:%s(%d),prob:%7.6f,size:%d,is_reversed:%d",
                         PredictionType_Name(type_), type_, probability_,
                         points_.size(), is_reversed_);
}

void PredictedTrajectory::FromProto(const PredictedTrajectoryProto& proto) {
  FromProto(0.0, proto);
}

void PredictedTrajectory::FromProto(double shift_time,
                                    const PredictedTrajectoryProto& proto) {
  probability_ = proto.probability();
  type_ = proto.type();
  index_ = proto.index();
  is_reversed_ = proto.is_reversed();
  intention_ = proto.intention();
  points_.reserve(proto.points_size());
  if (proto.points_size() > 0) {
    if (type_ == PT_STATIONARY) {
      const auto& pt = proto.points(0);
      for (int i = 0; i < kPredictionPointNum; ++i) {
        points_.emplace_back();
        points_.back().FromProto(pt);
        points_.back().set_t(i * kPredictionTimeStep);
      }
    } else {
      int i = 0;
      while (i < proto.points_size() && proto.points(i).t() < shift_time) {
        ++i;
      }
      for (int j = i; j < proto.points_size(); ++j) {
        auto& pt = points_.emplace_back(proto.points(j));
        pt.set_t(pt.t() - proto.points(i).t());
        pt.set_s(pt.s() - proto.points(i).s());
      }
    }
  }
}

void PredictedTrajectory::ToProto(PredictedTrajectoryProto* proto,
                                  bool compress_traj) const {
  proto->Clear();
  proto->set_probability(probability_);
  proto->set_type(type_);
  proto->set_index(index_);

  proto->set_is_reversed(is_reversed_);

  if (!points_.empty()) {
    if (type_ == PT_STATIONARY) {
      points_.front().ToProto(proto->add_points());
    } else {
      for (const auto& point : points_) {
        point.ToProto(proto->add_points());
      }
    }
  }
}

std::optional<PredictedTrajectoryPoint> PredictedTrajectory::EvaluateByTime(
    double t) const {
  if (points_.size() < 2) {
    return std::nullopt;
  }

  if (t > points_.back().t() || t < points_.front().t()) {
    return std::nullopt;
  }

  const auto it_lower = std::lower_bound(
      points_.begin(), points_.end(), t,
      [](const PredictedTrajectoryPoint& sp, double t) { return sp.t() < t; });

  if (it_lower == points_.end()) return points_.back();
  if (it_lower == points_.begin()) return points_.front();
  const auto& p0 = *(it_lower - 1);
  const auto& p1 = *it_lower;
  const double t0 = p0.t();
  const double t1 = p1.t();
  const double alpha = LerpFactor(t0, t1, t);
  TrajectoryPointProto p_res;
  p_res.set_t(t);
  p_res.mutable_pos()->set_x(Lerp(p0.pos().x(), p1.pos().x(), alpha));
  p_res.mutable_pos()->set_y(Lerp(p0.pos().y(), p1.pos().y(), alpha));
  p_res.set_theta(Lerp(p0.theta(), p1.theta(), alpha));
  p_res.set_kappa(Lerp(p0.kappa(), p1.kappa(), alpha));
  p_res.set_steer_angle(Lerp(p0.steer_angle(), p1.steer_angle(), alpha));
  p_res.set_s(Lerp(p0.s(), p1.s(), alpha));
  p_res.set_v(Lerp(p0.v(), p1.v(), alpha));
  p_res.set_a(Lerp(p0.a(), p1.a(), alpha));
  return PredictedTrajectoryPoint(
      planning::TrajectoryPointWithAcceleration(p_res));
}

}  // namespace prediction
}  // namespace e2e_noa
