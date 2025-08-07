#ifndef ST_PLANNING_PREDICTION_PREDICTION_UTIL
#define ST_PLANNING_PREDICTION_PREDICTION_UTIL

#include <algorithm>
#include <vector>

#include "constraint.pb.h"
#include "object/spacetime_object_trajectory.h"
#include "prediction.pb.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction.h"
#include "prediction/prediction_defs.h"

namespace e2e_noa {
namespace prediction {

inline bool IsStationaryTrajectory(const PredictedTrajectoryProto& traj) {
  return traj.type() == PredictionType::PT_STATIONARY;
}
inline bool IsStationaryTrajectory(
    const prediction::PredictedTrajectory& traj) {
  return traj.type() == PredictionType::PT_STATIONARY;
}

inline bool IsStationaryPrediction(const ObjectPredictionProto& pred) {
  return pred.trajectories().size() == 1 &&
         IsStationaryTrajectory(pred.trajectories(0));
}
inline bool IsStationaryPrediction(const ObjectPrediction& pred) {
  return pred.trajectories().size() == 1 &&
         IsStationaryTrajectory(pred.trajectories()[0]);
}

void ExtendPredictionTraj(
    const planning::SpacetimeObjectTrajectory& traj,
    planning::ConstraintProto::LeadingObjectProto* leading_obj);

bool RefineTrajByAcc(PredictedTrajectory* const trajectory,
                     const double curr_acc, const double acc_ts_sec);

}  // namespace prediction
}  // namespace e2e_noa

#endif
