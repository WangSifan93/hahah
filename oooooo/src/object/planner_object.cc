#include "object/planner_object.h"

#include <cmath>
#include <utility>

#include "math/geometry/util.h"
#include "prediction/prediction_util.h"
#include "trajectory_point.pb.h"
#include "util/perception_util.h"

DEFINE_bool(planner_object_large_vehicle_use_perception_api, false,
            "Whether to use object type to determine large vehilce.");

namespace e2e_noa {
namespace planning {

namespace {
Box2d BuildPerceptionBoundingBox(const ObjectProto& object,
                                 const Box2d& bounding_box) {
  if (object.has_bounding_box()) {
    return Box2d(object.bounding_box());
  } else {
    return bounding_box;
  }
}
}  // namespace

PlannerObject::PlannerObject(prediction::ObjectPrediction object_prediction)
    : prediction_(std::move(object_prediction)) {
  FromPrediction(prediction_);
}

void PlannerObject::FromPrediction(
    const prediction::ObjectPrediction& prediction) {
  object_proto_ = prediction.perception_object();

  pose_.set_s(0.0);
  pose_.set_t(0.0);

  constexpr double kEpsilon = 1e-10;
  const double theta = object_proto_.yaw();
  const Vec2d v_vec = Vec2dFromProto(object_proto_.vel());
  const Vec2d a_vec = Vec2dFromProto(object_proto_.accel());
  double v{0}, a{0};

  switch (object_proto_.type()) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_CYCLIST:
    case OT_TRICYCLIST: {
      const Vec2d heading = Vec2d::FastUnitFromAngle(theta);
      v = v_vec.Dot(heading);
      a = a_vec.Dot(heading);
      break;
    }
    case OT_UNKNOWN_STATIC:
    case OT_PEDESTRIAN:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE: {
      v = v_vec.norm();
      a = a_vec.norm();
      break;
    }
  }

  const double kappa =
      std::fabs(v) < kEpsilon ? 0.0 : object_proto_.yaw_rate() / v;
  const Vec2d pos = Vec2dFromProto(object_proto_.pos());

  pose_.set_pos(pos);
  pose_.set_theta(theta);
  pose_.set_kappa(kappa);
  pose_.set_a(a);
  pose_.set_v(v);
  velocity_ = v_vec;

  contour_ = ComputeObjectContour(object_proto_);

  bounding_box_ = contour_.BoundingBoxWithHeading(theta);

  perception_bbox_ = BuildPerceptionBoundingBox(object_proto_, bounding_box_);

  aabox_ = contour_.AABoundingBox();

  is_stationary_ = prediction_.trajectories().empty() ||
                   prediction::IsStationaryPrediction(prediction_) ||
                   v < kEpsilon;

  is_large_vehicle_ = FLAGS_planner_object_large_vehicle_use_perception_api
                          ? (object_proto_.type() == OT_LARGE_VEHICLE)
                          : IsLargeVehicle(object_proto_);

  if (const auto found = object_proto_.id().find("(ICCAgent)");
      found != std::string::npos) {
    is_sim_agent_ = true;
    base_id_ = object_proto_.id().substr(0, found);
  }
}

std::optional<int> PlannerObject::MostLikelyTrajectory() const {
  const int n = num_trajs();
  if (n == 0) return std::nullopt;
  double max_prob = 0.0;
  int traj_index = 0;
  for (int i = 0; i < n; ++i) {
    if (traj(i).probability() > max_prob) {
      max_prob = traj(i).probability();
      traj_index = i;
    }
  }
  return traj_index;
}

void PlannerObject::ToPlannerObjectProto(
    PlannerObjectProto* planner_object_proto) const {
  planner_object_proto->mutable_object()->CopyFrom(object_proto_);
  planner_object_proto->set_is_stationary(is_stationary_);
  const auto& trajs = prediction_.trajectories();
  const int num_trajs = trajs.size();

  for (int i = 0; i < num_trajs; ++i) {
    const auto& traj = trajs[i];

    auto prediction_info = planner_object_proto->add_prediction();
    prediction_info->set_probability(traj.probability());

    const auto& points = traj.points();
    const int num_points = points.size();
    for (int j = 0; j < num_points; ++j) {
      const auto& pt = points[j];
      auto pose = prediction_info->add_points();
      Vec2dToProto(pt.pos(), pose->mutable_pos());
      pose->set_s(pt.s());
      pose->set_theta(pt.theta());
      pose->set_kappa(pt.kappa());
      pose->set_t(pt.t());
      pose->set_v(pt.v());
      pose->set_a(pt.a());
    }
  }
}

}  // namespace planning
}  // namespace e2e_noa
