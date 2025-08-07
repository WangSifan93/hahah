#pragma once

#include "context/e2e_planner_context.h"
#include "context/ego_state.h"
#include "context/lane_change_command_update.h"
#include "context/lane_manager.h"

namespace e2e_noa {
namespace spt {

struct LateralSolverOption {
  bool enable_log{false};
  bool loose_obstacle_bound{false};
  bool use_raw_model_traj{false};
  bool use_refined_path{false};
  bool bind_end_state{true};
};
class SpationTemporal {
 public:
  explicit SpationTemporal(
      const PathConfigBucket *config_bucket,
      const std::shared_ptr<PathTaskPipelineContext> &pipeline_context);

  virtual ~SpationTemporal() = default;

  bool execute(Frame *frame) override;

 protected:
  bool CheckReflineValid(const ilqr::LatMotionPlanInput &input) const;
  void GetDensePathSAndL(
      const std::vector<State> &dense_states,
      const std::shared_ptr<FrenetCoordinateSystem> &ref_path,
      std::vector<double> *dense_path_s_list);
  void SetOutputSList(const std::shared_ptr<FrenetCoordinateSystem> &ref_path,
                      path_planner::ProtoStates *output);
  bool CheckRet(const TrajectoryPoints &ret);
  static void UpdateLatInfo(
      const std::map<LaneLineType, std::vector<OBB>> &obb_map,
      const PPPreprocessorInput &input, const PPPreprocessorOutput &output,
      path_planner::TrajectoryTtcAssessor *const traj_ttc_assessor);
  static void UpdateResPathArcTtcInfos(
      path_planner::TrajectoryTtcAssessor *const traj_ttc_assessor,
      const PPPreprocessorInput &input,
      const path_planner::ProtoStates &input_path, const ObbBoxs *obb_boxs,
      CollisionRiskInfo *col_risk_info);
  bool IsObsFront(const State &state, const std::vector<Vec2d> &obs_corners,
                  const AgentParams &agent_params) const;
  void IsCurrentEgoOverlapping(const AgentParams &agent_params,
                               const State &current_ego_state, bool &is_overlap,
                               bool &is_vru);
  void CalLimitS(
      const AgentParams &agent_params, const DensePath &dense_path,
      const ::google::protobuf::RepeatedPtrField<obstacle::ObjectWallInfo>
          &consider_obs,
      ilqr::ObsOverlapInfo *obs_overlap_info);
  void UpdateWarmStartData(const LatIlqrData &prev_ilqr_data,
                           ilqr::WarmStartData *warm_start_data);
  void UpdateObstacleAvoidInfo(const path_plan_pre::PPPreprocessorInput &input,
                               const bool is_interact_state);
  void CheckShortRangeIntRequest(
      const AgentParams &agent_params,
      const path_planner::ProtoStates &final_path,
      const ::google::protobuf::RepeatedPtrField<obstacle::ObjectWallInfo>
          &consider_obs,
      const ObbBoxs &rb_boxs);
  const PlanningInitPoint &init_point() const {
    return reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  }

 protected:
  const SpationTemporalConfig *config_{nullptr};
  Frame *frame_{nullptr};
};

}  // namespace spt
}  // namespace e2e_noa
