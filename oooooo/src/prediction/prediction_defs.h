#ifndef ONBOARD_PREDICTION_PREDICTION_DEFS_H_
#define ONBOARD_PREDICTION_PREDICTION_DEFS_H_

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "math/geometry/box2d.h"
#include "math/piecewise_linear_function.h"
#include "math/vec.h"
#include "perception.pb.h"
#include "router/plan_passage.h"
namespace e2e_noa {
namespace prediction {

using ResampledObjectsHistory = std::vector<std::vector<e2e_noa::ObjectProto>>;

using ObjectIDType = std::string;
using ProbTrajPair = std::pair<double, std::vector<Vec2d>>;

using ObjectProbTrajs = std::vector<ProbTrajPair>;
using ObjectsProbTrajs = absl::flat_hash_map<std::string, ObjectProbTrajs>;

enum class PredictTypePrio { HIGH, MED, LOW };
using TypePrioMap =
    std::map<PredictTypePrio, const absl::flat_hash_set<ObjectType>>;

struct ObjectSpacetimeState {
  double timestamp;
  Vec2d pos;
  double heading;
  Vec2d vel;
  Box2d bbox;

  std::string DebugString() const {
    return absl::StrFormat("timestamp: %.6f, pos: %s, heading: %.6f, vel: %s.",
                           timestamp, pos.DebugString(), heading,
                           vel.DebugString());
  }
};

struct ObjectSpacetimeHistory {
  ObjectIDType id;
  ObjectType type;
  std::vector<ObjectSpacetimeState> states;
};
using ObjectsSpacetimeHistory = std::vector<ObjectSpacetimeHistory>;

using NLLTrajPoint = std::array<double, 5>;
struct AgentCentricObjectProbTraj {
  double mode_prob = 0.0;
  std::array<double, 3> relation_probs = {0.0, 0.0, 0.0};
  std::vector<NLLTrajPoint> traj_points;
  double rot_rad = 0.0;
};
using AgentCentricObjectProbTrajs = std::vector<AgentCentricObjectProbTraj>;
using AgentCentricObjectsProbTrajs =
    absl::flat_hash_map<std::string, AgentCentricObjectProbTrajs>;

struct AgentCentricObjectOut {
  AgentCentricObjectProbTrajs prob_trajs;
  std::optional<double> startup_prob;
};
using AgentCentricObjecstOut =
    absl::flat_hash_map<std::string, AgentCentricObjectOut>;

using CutinTrajPoint = std::array<double, 2>;
struct CutinObjectProbTraj {
  double mode_prob = 0.0;
  std::vector<CutinTrajPoint> traj_points;
  double rot_rad = 0.0;
};

using CutinObjectProbTrajs = std::vector<CutinObjectProbTraj>;
using CutinObjectsProbTrajs =
    absl::flat_hash_map<std::string, CutinObjectProbTrajs>;

struct CutinObjectOut {
  CutinObjectProbTrajs prob_trajs;
  std::vector<double> channle_probs;
  int predicted_channel;
  int cur_channel;
};

using CutinObjectsOut = absl::flat_hash_map<std::string, CutinObjectOut>;

struct CutinSLObjectOut {
  std::vector<double> channle_probs;
  int predicted_channel;
  int cur_channel;
};
using CutinSLObjectsOut = absl::flat_hash_map<std::string, CutinSLObjectOut>;

struct LaneSelectionObjectOut {
  std::map<planning::PlanPassage, double> scores_for_dps;
};
using LaneSelectionObjectsOut =
    absl::flat_hash_map<std::string, LaneSelectionObjectOut>;

struct AgentInfoWithDps {
  ObjectIDType id;
  std::vector<planning::PlanPassage> plan_passages;
  std::vector<float> dp_scores;
};

constexpr double kEpsilon = 1e-8;

constexpr char kAvObjectId[] = "AV";
constexpr char kInvalidObjectId[] = "NA";

constexpr double kFeatureV2HistoryStepNum = 21;
constexpr double kLaneSelectionHistoryStepNum = 10;
constexpr double kFeatureV2HistoryStepLen = 0.1;
constexpr double kFeatureV2MaxMapSampleLen = 10.0;
constexpr int kFeatureV2MapSegmentNum = 5;

constexpr double kPredictionTimeStep = 0.1;
constexpr double kPredictionDuration = 8.0;
constexpr double kCutinPredictionDuration = 5.0;
const int kPredictionPointNum =
    static_cast<int>(kPredictionDuration / kPredictionTimeStep);

constexpr double kEmergencyGuardHorizon = 3.0;
constexpr double kSafeHorizon = 8.0;
constexpr double kComfortableHorizon = 8.0;

constexpr double kVehCurvatureLimit = 0.25;
constexpr double kVehLateralAccelLimit = 2.5;

constexpr double kHistoryLen = 1.5;

constexpr double kTooShortTrajLen = 0.5;

constexpr double kAccelerationFitTime = 1.0;
constexpr double kMaintainAccTime = 1.5;

constexpr int kActNetJ5ModelTrajNum = 2;

constexpr int kPolyFitDownSampleStep = 3;

const std::vector<double> kObjectLengthDataPoint = {1.0,   1.57,  2.95,
                                                    4.786, 5.995, 10.48};
const PiecewiseLinearFunction<double, double> kLengthToWheelbasePlf(
    kObjectLengthDataPoint,
    std::vector<double>{0.6, 1.22, 2.1, 2.92, 3.85, 5.89});
const PiecewiseLinearFunction<double, double> kLengthToMaxFrontSteerPlf(
    kObjectLengthDataPoint,
    std::vector<double>{M_PI / 3.0, 0.49, 0.496, 0.546, 0.583, 0.67});
struct AVObjectRelation {
  double no_relation;
  double yield;
  double pass;

  std::string DebugString() const {
    return absl::StrFormat("void: %f, yield: %f, pass: %f", no_relation, yield,
                           pass);
  }
};

using ProbTrajPair = std::pair<double, std::vector<Vec2d>>;

using ObjectProbTrajs = std::vector<ProbTrajPair>;
using ObjectsProbTrajs = absl::flat_hash_map<std::string, ObjectProbTrajs>;
using NLLTrajPoint = std::array<double, 5>;

using AgentCentricObjectProbTrajs = std::vector<AgentCentricObjectProbTraj>;
using AgentCentricObjectsProbTrajs =
    absl::flat_hash_map<std::string, AgentCentricObjectProbTrajs>;

using AgentCentricObjecstOut =
    absl::flat_hash_map<std::string, AgentCentricObjectOut>;

using CutinSLObjectsOut = absl::flat_hash_map<std::string, CutinSLObjectOut>;

}  // namespace prediction
}  // namespace e2e_noa

#endif
