#ifndef MAP_LANE_SEQUENCE_H
#define MAP_LANE_SEQUENCE_H
#include <limits>
#include <vector>

#include "maps/lane.h"
#include "maps/map_def.h"

namespace ad_e2e {
namespace planning {
class LaneSequence {
 public:
  explicit LaneSequence(const std::vector<LaneConstPtr> &lanes_ptr_vec);
  ~LaneSequence() = default;
  const std::vector<LaneConstPtr> &lanes() const { return lanes_; }
  std::vector<LaneConstPtr> &mutable_lanes() { return lanes_; }

  LaneConstPtr front_lane() const;

  bool IsValid();

  double GetTrueLength() const;
  double GetPointsLength() const;

  bool SamplePoints(double start_s, std::vector<Point2d> *const points,
                    double interval = 3.0) const;

  bool AddLane(const LaneConstPtr &lane);

  math::Vec2d GetPointAtS(double s) const;

  double GetProjectionDistance(const double &x, const double &y) const;

  double GetProjectionDistance(const math::Vec2d &point) const;

  LaneConstPtr GetNearestLane(const math::Vec2d &point,
                              double *dis = nullptr) const;

  double GetProjectionDistance(const math::Vec2d &point, double *s_offset,
                               double *l_offset = nullptr) const;

  double GetDistanceToPOI(const PoiType &poi_type, const double &x,
                          const double &y) const;

  double GetDistanceToPOI(
      const PoiType &poi_type, const double &x, const double &y,
      LaneConstPtr &poi_lane,
      const ad_e2e::planning::NaviPosition *navi_start = nullptr) const;

  void GetBoundaryTypeFromS(const bool &is_left, const double &s,
                            std::vector<LaneBoundaryType> &boundary_type) const;

  void GetBoundaryTypeFromPoint(
      const bool &is_left, const math::Vec2d &point,
      std::vector<LaneBoundaryType> &boundary_type) const;
  void GetRoadSegments(const bool is_left,
                       std::vector<math::LineSegment2d> *road_segs) const;

  double GetDistanceToJunction(LaneConstPtr &poi_lane,
                               const ad_e2e::planning::NaviPosition &navi_start,
                               bool end_check = false,
                               bool special_check = false) const;

  double GetDistanceBetweenJunction(const std::string &begin_junction_id,
                                    LaneConstPtr &next_virtual_lane) const;
  double GetDistanceToJunction(const double &x, const double &y,
                               LaneConstPtr &poi_lane) const;

  double GetDistanceExitJunction(
      const ad_e2e::planning::NaviPosition &navi_start) const;

  bool CanPassJunction(const ad_e2e::planning::NaviPosition &navi_start,
                       double *dist_to_junction = nullptr,
                       bool next_junction = false);
  bool IsTurnLeft(const double &x, const double &y);
  bool IsTurnRight(const double &x, const double &y);

  bool IsOnLaneSequence(const LaneConstPtr &lane) const;
  const LanePtr GetPreLaneOnLaneSequence(const LaneConstPtr &lane) const;
  const LanePtr GetNextLaneOnLaneSequence(const LaneConstPtr &lane) const;

  double GetDistanceToTargetLaneType(
      LaneConstPtr &poi_lane, const std::unordered_set<LaneType> &lane_type_set,
      const ad_e2e::planning::NaviPosition &navi_start) const;
  double GetDistanceToCustomSpeedLimit(
      const ad_e2e::planning::NaviPosition &navi_start,
      double *speed_limit) const;

 private:
  std::vector<LaneConstPtr> lanes_;
};

class NavigableLaneSequence : public LaneSequence {
 public:
  explicit NavigableLaneSequence(
      const std::vector<LaneConstPtr> &lanes_ptr_vec);
  ~NavigableLaneSequence() = default;
  void SetNaviDistance(const float navi_dis) { navi_distance_ = navi_dis; }
  void SetNearestObstacleVel(const float nearest_obstacle_vel) {
    nearest_obstacle_vel_ = nearest_obstacle_vel;
  }
  void SetNearestObstacleDistance(const float nearest_obstacle_distance) {
    nearest_obstacle_distance_ = nearest_obstacle_distance;
  }
  void SetDistanceToEgoCar(const float distance_to_ego_car) {
    distance_to_ego_car_ = distance_to_ego_car;
  };
  void SetSequenceIdx(const float sequence_idx) {
    sequence_idx_ = sequence_idx;
  }
  void SetIsNaviSequence(const bool is_navi_sequence) {
    is_navi_sequence_ = is_navi_sequence;
  }
  void SetProbability(const float probability) { probability_ = probability; }
  void SetLcReason(const LcReason lc_reason) { lc_reason_ = lc_reason; }
  void SetDebugContent(const std::string &debug_content) {
    debug_content_ += debug_content;
  }
  void SetProjectionPoint(const Point2d &projection_point) {
    projection_point_ = projection_point;
  }
  void SetIsOccupyLaneSeq(bool is_occupy_lane_seq) {
    is_occupy_lane_seq_ = is_occupy_lane_seq;
  }
  void SetIsOptimalLaneSeq(bool is_optimal_lane_seq) {
    is_optimal_lane_seq_ = is_optimal_lane_seq;
  }
  void SetCompleteSequences(
      std::vector<std::vector<LaneConstPtr>> &complete_sequences) {
    complete_sequences_ = complete_sequences;
  }
  void SetNearestObstacle(const ObstacleForLane &obstacle) {
    nearest_obstacle_ = obstacle;
  }
  void SetRightRoundaboutLanesNum(
      const std::int32_t right_roundabout_lanes_num) {
    right_roundabout_lanes_num_ = right_roundabout_lanes_num;
  }
  void SetRightLanesNum(const std::int32_t right_lanes_num) {
    right_lanes_num_ = right_lanes_num;
  }
  void SetScoreInStraightJunction(const float score_in_straight_junction) {
    score_in_straight_junction_ = score_in_straight_junction;
  }
  void AddObstacleInSequence(
      const std::pair<std::string, double> &obstacle_in_sequence) {
    obstacles_in_sequence_.emplace_back(obstacle_in_sequence);
  }
  const float GetNaviDistance() { return navi_distance_; }
  const float GetSequenceIdx() { return sequence_idx_; }
  const float GetNearestObstacleVel() { return nearest_obstacle_vel_; }
  const float GetNearestObstacleDistance() {
    return nearest_obstacle_distance_;
  }
  const float GetDistanceToEgoCar() const { return distance_to_ego_car_; }
  const float GetProbability() const { return probability_; }
  const LcReason &GetLcReason() const { return lc_reason_; }
  bool IsOverLap(const std::vector<LaneConstPtr> &other_lane_sequence);
  bool IsTwoSplitLane(const std::vector<LaneConstPtr> &other_lane_sequence);
  const bool GetIsNaviSequence() { return is_navi_sequence_; }
  const bool GetIsOccupyLaneSeq() { return is_occupy_lane_seq_; }
  const bool GetIsOptimalLaneSeq() { return is_optimal_lane_seq_; }
  const std::int32_t GetRightRoundaboutLanesNum() {
    return right_roundabout_lanes_num_;
  }
  const std::int32_t GetRightLanesNum() { return right_lanes_num_; }
  const std::vector<std::pair<std::string, double>> GetObstaclesInSequence() {
    return obstacles_in_sequence_;
  }
  const std::string GetDebugContent() { return debug_content_; }
  const Point2d GetProjectionPoint() { return projection_point_; }
  const std::vector<std::vector<LaneConstPtr>> GetCompleteSequences() {
    return complete_sequences_;
  }
  const ObstacleForLane GetNearestObstacle() { return nearest_obstacle_; }
  const float GetScoreInStraightJunction() {
    return score_in_straight_junction_;
  }
  std::string FindSamePrevLane(
      const std::vector<LaneConstPtr> &other_lane_sequence);

 private:
  bool is_occupy_lane_seq_ = false;
  bool is_optimal_lane_seq_ = false;
  float navi_distance_ = 1000.0;
  float sequence_idx_ = 0.0;
  float nearest_obstacle_vel_ = 0.0;
  float nearest_obstacle_distance_ = 150;
  float distance_to_ego_car_ = 0.0;
  float score_in_straight_junction_ = 0.0;
  bool is_navi_sequence_ = false;
  std::int32_t right_roundabout_lanes_num_ = -1;
  std::int32_t right_lanes_num_ = -1;
  std::string debug_content_;
  Point2d projection_point_;
  std::vector<std::vector<LaneConstPtr>> complete_sequences_;
  std::vector<std::pair<std::string, double>> obstacles_in_sequence_;
  float probability_ = 0.0;
  LcReason lc_reason_ = LcReason::LC_REASON_NONE;
  ObstacleForLane nearest_obstacle_;
};

typedef std::shared_ptr<NavigableLaneSequence> NavigableLaneSequencePtr;
typedef std::shared_ptr<LaneSequence> LaneSequencePtr;

using CandidateSequencesPtr =
    std::shared_ptr<std::vector<NavigableLaneSequencePtr>>;
using CandidateSequencesConstPtr =
    std::shared_ptr<const std::vector<NavigableLaneSequencePtr>>;

enum BehaviorCommand {
  Command_Invalid = 0,
  Command_LaneChangeLeft = 1,
  Command_LaneChangeRight = 2,
  Command_LaneChangeCancel = 3
};
struct LaneSeqInfo {
  SectionInfo cur_section;
  int cur_section_lane_num = 0;
  int cur_lane_position = -1;

  LaneSequencePtr lane_seq = nullptr;
  LaneConstPtr nearest_lane = nullptr;
  LaneConstPtr navi_end_lane = nullptr;
  double dist_to_navi_end = std::numeric_limits<double>::infinity();
  bool lane_seq_connect_navi_end = false;
  bool is_current = false;

  double dist_to_left_solid_line = std::numeric_limits<double>::infinity();
  double dist_to_right_solid_line = std::numeric_limits<double>::infinity();

  double dist_to_junction = std::numeric_limits<double>::infinity();
  double dist_to_junction_id = std::numeric_limits<double>::infinity();
  BehaviorCommand navi_lc_command = Command_Invalid;
  int lc_num = 0;

  double dist_to_bus_lane = std::numeric_limits<double>::infinity();

  int pnp_top1_lc_reason = 0;

  double dist_to_merge = std::numeric_limits<double>::infinity();
  double dist_to_split = std::numeric_limits<double>::infinity();

  double dist_to_junction_v2 = std::numeric_limits<double>::infinity();
  double dist_to_navi_end_v2 = std::numeric_limits<double>::infinity();
  ad_e2e::planning::V2TurnInfo::V2DetailTurnType nearest_turn_type_v2 =
      ad_e2e::planning::V2TurnInfo::V2DetailTurnType::NONE;
  ad_e2e::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2 =
      ad_e2e::planning::V2TurnInfo::V2DetailTurnType::NONE;
  ad_e2e::planning::V2TurnInfo::V2TurnType turn_type_v2 =
      ad_e2e::planning::V2TurnInfo::V2TurnType::UNKNOWN;
};
typedef std::shared_ptr<LaneSeqInfo> LaneSeqInfoPtr;
}  // namespace planning
}  // namespace ad_e2e

#endif
