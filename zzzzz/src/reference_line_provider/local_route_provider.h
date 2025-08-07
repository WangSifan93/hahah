//
// Copyright 2024 zpilot. All Rights Reserved.
//

/**
 * @file local_route_provider.h
 *
 * @brief Declaration of the class LocalRouteProvider.
 */

#pragma once

#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "apps/planning/src/common/local_route/local_route.h"
#include "apps/planning/src/common/vehicle_state/proto/vehicle_state.h"
#include "apps/planning/src/planning_msgs/navigation.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/planning_msgs/local_route_config.h"
#include "apps/planning/src/reference_line_provider/pnc_map/pnc_map.h"
#include "apps/planning/src/reference_line_provider/map_polynomial_fit/map_polynomial_fit.h"
#include "apps/planning/src/reference_line_provider/smoothers/discrete_points_local_route_smoother.h"
#include "messages/map_service/all_map_new.pb.h"
#include "apps/planning/src/reference_line_provider/acc/acc_trajectory_builder.h"

/**
 * @namespace zark::planning
 * @brief zark::planning
 */
namespace zark {
namespace planning {

constexpr double kMinBoundValue = 1.0e-6;
constexpr double kMinSampleDist = 1.0;     //[m]
constexpr double kNOALineSafeDist = 0.85;  //[m]
constexpr double kHostLineSafeDist = 0.7;  //[m]
constexpr int kLineFitteOrder = 5;
constexpr int kEdgeFitteOrder = 3;

/**
 * @class LocalRouteProvider
 * @brief The class of LocalRouteProvider.
 *        It provides smoothed local route to planning.
 */
class LocalRouteProvider {
 public:
  using LocalRouteList = std::list<std::shared_ptr<LocalRoute>>;
  using LocalRouteHistory = std::queue<LocalRouteList>;
  using RouteSegmentsList = std::list<std::shared_ptr<hdmap::RouteSegments>>;
  using RouteSegmentsHistory = std::queue<RouteSegmentsList>;

 public:
  struct SpecialLaneInfo {
    SpecialLaneInfo()
        : split_in_left(false),
          split_in_right(false),
          left_is_merge(false),
          right_is_merge(false) {}

    bool split_in_left;   // this lane split in road left side
    bool split_in_right;  // this lane split in road right side
    bool left_is_merge;   // left neighbor lane is merge lane
    bool right_is_merge;  // right neighbor lane is merge lane
  };

  LocalRouteProvider();

  // thread start
  bool Start();

  // thread stop
  void Stop();

  // get local routes for planning
  bool GetLocalRoutes(const ::common::TrajectoryPoint& planning_start_point,
                      const bool is_acc_mode,
                      std::list<LocalRoute>& local_routes);

  // get time-consuming of local route generation algorithm
  const double LastTimeDelay();

  /**
   * @brief check local_route_provider Normally generation of local_routes
   * @return true if generate local_routes successfully
   **/
  inline bool IsLocalRouteUpdated() {
    std::lock_guard<std::mutex> lock(local_routes_mutex_);
    return is_local_route_updated_;
  }

  inline ::math::Vec2d GetWidthPoint(const LocalRoutePoint& center_point,
                                     const double side_width) {
    return ::math::Vec2d(
        center_point.x() - side_width * std::sin(center_point.heading()),
        center_point.y() + side_width * std::cos(center_point.heading()));
  }

  inline ::math::Vec2d GetWidthPoint(const ::math::Vec2d& center_point,
                                     const double side_width,
                                     const double heading) {
    return ::math::Vec2d(center_point.x() - side_width * std::sin(heading),
                         center_point.y() + side_width * std::cos(heading));
  }

  inline ::math::Vec2d MoveWidthPoint(const LocalRoutePoint& center_point,
                                      const double move_dist,
                                      const double heading) {
    return ::math::Vec2d(center_point.x() - move_dist * std::sin(heading),
                         center_point.y() + move_dist * std::cos(heading));
  }

  const std::pair<uint64_t, DEC_Outputs> GetFittedLines() {
    DEC_Outputs empty_result;
    uint64_t time_now = 0;
    std::lock_guard<std::mutex> lock(fitted_line_mutex_);
    if (fitted_line_result_.size() == 0) {
      return {time_now, empty_result};
    }
    return fitted_line_result_.back();
  }

  void ClearFittedLine() {
    {
      std::lock_guard<std::mutex> lock(local_routes_mutex_);
      is_local_route_updated_ = false;
    }
    fitted_line_result_.clear();
    refline_start_time_ = 0;
    fix_noa_line_.first.ClearFixInfo();
    fix_noa_line_.second.clear();
    last_move_info_.ResetMoveInfo();
    NavigationInfo error_info;
    DataReader::GetInstance()->SetNaviResult(error_info);
    DEC_Outputs empty_result;
    DataReader::GetInstance()->SetFittedResult({0, empty_result});
    reference_line_proto::ReferenceLineState refline_state;
    refline_state.set_state(reference_line_proto::ReferenceLineState::State::
                                ReferenceLineState_State_ERROR_STATE);
    refline_state.set_error_msg(error_msg_);
    DataReader::GetInstance()->SetRefLineState(refline_state);
    history_noa_lines_.clear();
  }

  bool PostHeadingProcess(const std::vector<LocalRoutePoint>& xy_points,
                          std::vector<double>& headings);

 private:
  /**
   * @brief Use PncMap to create local route and the corresponding segments
   * based on routing and current position. This is a thread safe function.
   * @return true if !local_routes.empty() && local_routes.size() ==
   *                 segments.size();
   **/
  bool CreateLocalRoute(const common::VehicleState& vehicle_state,
                        std::list<LocalRoute>& local_routes,
                        std::list<hdmap::RouteSegments>& segments);

  /**
   * @brief store the computed local route. This function can avoid
   * unnecessary copy if the reference lines are the same.
   */
  void UpdateLocalRoute(const std::list<LocalRoute>& local_routes,
                        const std::list<hdmap::RouteSegments>& route_segments);

  void GenerateThread();

  bool CreateRouteSegments(const common::VehicleState& vehicle_state,
                           std::list<hdmap::RouteSegments>& segments);

  bool IsLocalRouteSmoothValid(const LocalRoute& raw,
                               const LocalRoute& smoothed) const;

  bool SmoothLocalRoute(const LocalRoute& raw_local_route,
                        LocalRoute& local_route,
                        const bool& need_smooth = true);

  /**
   * @brief when noaline stop trigger, keep alone last noaline smooth local
   * route until ego car crossed the endpoint for local route consistency.
   */
  bool SmoothCurrentLocalRoute(const LocalRoute& raw_local_route,
                               const LocalRoute& last_noaline,
                               LocalRoute& local_route);

  bool SmoothPrefixedLocalRoute(const LocalRoute& prefix_ref,
                                const LocalRoute& raw_ref,
                                LocalRoute& local_route);

  bool CalFixNOALine(
      const LocalRoute& raw_reference_line,
      const hdmap::RouteSegments& segments, const int8_t noaline_dir,
      std::pair<FixNoaLineInfo, std::vector<LocalRoutePoint>>& fix_noa_line,
      LocalRoute* local_route);

  /**
   * @brief when noaline stop trigger, check if ego car has crossed the endpoint
   * of the previous noaline.
   * @return false if ego car has crossed the endpoint, else return true.
   */
  bool FollowNOALine(std::unordered_map<std::string, FixNoaLineInfo>& noalines,
                     LocalRoute& follow_local_route);

  bool SetDifferentBound(const map_service::zmap::LaneInfoConstPtr& lane_info,
                         const ::math::Vec2d& route_point,
                         const double safe_dist, double& move_dist);

  void GetAnchorPoints(const LocalRoute& local_route,
                       std::vector<RouteAnchorPoint>& anchor_points) const;

  void GetAnchorPointsWithLastRef(
      const LocalRoute& local_route, const LocalRoute& last_ref_route,
      std::vector<RouteAnchorPoint>& anchor_points,
      std::vector<LocalRoutePoint>& last_ref_points,
      std::pair<uint32_t, uint32_t>& ref_index_range);

  void GetNOALineAnchorPoints(
      std::pair<FixNoaLineInfo, std::vector<LocalRoutePoint>>& fix_noa_line,
      const LocalRoute& local_route, const hdmap::RouteSegments& segments,
      const int8_t noaline_dir, std::vector<RouteAnchorPoint>* anchor_points,
      std::vector<LocalRoutePoint>& last_ref_points,
      std::pair<uint32_t, uint32_t>& ref_index_range);

  bool AddLastSimilarity(const LocalRoute& local_route,
                         LocalRoute& last_ref_route, double& forward_ref_s,
                         double& backward_ref_s);

  void GetSufficientDistPoint(const LocalRoutePoint& original_point,
                              const LocalRoutePoint& fix_point,
                              const double safe_dist, RouteAnchorPoint& anchor);

  void ReSampleRefPoints(const LocalRoute& local_route,
                         std::vector<LocalRoutePoint>& ref_points);

  bool SmoothRouteSegment(const hdmap::RouteSegments& segments,
                          LocalRoute& local_route,
                          const bool& need_smooth = true);

  /**
   * @brief This function creates a smoothed forward local route
   * based on the given segments.
   */
  bool ExtendLocalRoute(const common::VehicleState& state,
                        hdmap::RouteSegments& segments,
                        LocalRoute& local_route);

  RouteAnchorPoint GetAnchorPoint(const LocalRoute& local_route,
                                  double s) const;

  bool Shrink(const ::common::SLPoint& sl, LocalRoute& ref,
              hdmap::RouteSegments& segments);

  void ThreadSleep(const double start_time, const double end_time);

  bool FitReferenceLine(const std::list<LocalRoute>& reference_lines,
                        const std::list<hdmap::RouteSegments>& segments,
                        DEC_Outputs& fit_result);

  void ProcessBoundaryLine(const LocalRoute& refline,
                           const hdmap::RouteSegments& segment,
                           const MapPolynomialFit::FittedLine& center_line,
                           const double ego_s,
                           const SpecialLaneInfo& split_merge_info,
                           B1_LaneLineInfo& left_bound_line,
                           B1_LaneLineInfo& right_bound_line);

  bool FindSplit(const LocalRoute& current_line,
                 const hdmap::RouteSegments& segment,
                 SpecialLaneInfo& split_merge_info);

  void ProcessSplitBoundary(
      const MapPolynomialFit::FittedLine& center_line,
      const MapPolynomialFit::FittedLine& oppsite_fitted_line,
      MapPolynomialFit::FittedLine& bound_line);

  void ProcessMergeBoundary(
      const hdmap::RouteSegments& segment,
      const SpecialLaneInfo& split_merge_info,
      const MapPolynomialFit::FittedLine& oppsite_fitted_line,
      MapPolynomialFit::FittedLine& bound_line);

  void FillOutputLine(const MapPolynomialFit::FittedLine& fit_line,
                      zark::ads_common::B1_LaneLineInfo& out_line);

  void FillOutputLine(const MapPolynomialFit::FittedLine& fit_line,
                      zark::ads_common::B_RoadEdge_st& out_line);

  void ExtractBoudaryPoint(
      const LocalRoute& refline, const hdmap::RouteSegments& segment,
      const double ego_s, const SpecialLaneInfo& split_merge_info,
      std::pair<hdmap_new::LaneBoundaryAttribute,
                hdmap_new::LaneBoundaryAttribute>& line_type,
      std::pair<std::vector<LocalRoutePoint>, std::vector<LocalRoutePoint>>&
          boundary_points,
      int& move_base);

  void ExtractEdgePoint(const LocalRoute& refline,
                        const hdmap::RouteSegments& segment,
                        std::vector<LocalRoutePoint>& left_points,
                        std::vector<LocalRoutePoint>& right_points);

  void FillBoundaryType(const hdmap_new::LaneBoundaryAttribute& boundary_type,
                        zark::ads_common::B1_LaneLineInfo& out_line);

  void ProcessRoadEdge(const LocalRoute& refline,
                       const hdmap::RouteSegments& segment,
                       B_RoadEdge_st& left_edge, B_RoadEdge_st& rigth_edge);

  void UpdateHostLaneChangeInfo(DEC_Outputs& fitted_result);

  bool GetLastRefRoute(const hdmap::RouteSegments& segments,
                       LocalRoute& last_ref_route);

  inline void CalculateRefDis(const float lane_seg_length, float& ref_start_s,
                              float& ref_end_s, float& accumulate_s) {
    ref_start_s = accumulate_s;
    accumulate_s += lane_seg_length;
    ref_end_s = accumulate_s;
  }

  void GetMapBoundaryPoint(
      const map_service::zmap::LaneBoundaryInfoConstPtr& boundary_info,
      const double start_s, const double end_s,
      const std::pair<bool, float>& move_info,
      std::vector<LocalRoutePoint>& extract_point,
      const bool& is_first_seg = false);

  void CurveNodeProcess(const NavigationInfo& nav_info,
                        const hdmap::RouteSegments& segment,
                        const double ego_map_s, DEC_Outputs& decision_result);

  void ExtractLaneCurveNode(const NavSection& section_info,
                            const float ego_path_offset,
                            std::vector<EMMsg_Curv_Node>& msg_curv_nodes,
                            float& distance_ego, std::string& target_id);

  void ExtractLaneCurveNode(
      const zark::map_service::zmap::LaneInfoConstPtr map_lane_info,
      const float ego_path_offset, std::vector<EMMsg_Curv_Node>& msg_curv_nodes,
      float& distance_ego);

  void FillCurvNodes(const std::vector<EMMsg_Curv_Node>& msg_curv_nodes,
                     DEC_Outputs& decision_result);

  /**
   * @brief This function check forward lane width and return lateral move
   * direction if lane width error
   *
   * @return 0 if lane width is not exceed max width; return 1, lane width error
   * and need use left boundary move to replace right boundary; return -1, lane
   * width error and need use riht boundary move to replace left boundary.
   */
  int CheckLaneWidth(const LocalRoute& refline,
                     BoundaryMoveInfo& last_move_info);

  bool GetBoundaryDistance(const LocalRoutePoint& ref_point, double& left_l,
                           double& right_l);

  bool GetBoundaryDistance(const ::math::Vec2d& center_point,
                           const std::string& lane_id, double& left_l,
                           double& right_l);

  bool GetBoundaryDistance(
      const ::math::Vec2d& center_point,
      const std::pair<map_service::zmap::LaneBoundaryInfoConstPtr,
                      map_service::zmap::LaneBoundaryInfoConstPtr>&
          boundary_infos,
      double& left_l, double& right_l);

  bool GetBoundaryInfo(
      const std::string& lane_id,
      std::pair<map_service::zmap::LaneBoundaryInfoConstPtr,
                map_service::zmap::LaneBoundaryInfoConstPtr>& boundary_infos);

  bool SearchMeetWidthPoint(const std::string& start_lane_id,
                            const std::string& first_split_id,
                            const ::math::Vec2d& start_point,
                            FixNoaLineInfo& fix_info,
                            LocalRoutePoint& target_point);

  std::pair<bool, float> GetBoundaryMoveInfo(
      const map_service::zmap::LaneBoundaryInfoConstPtr& map_boundary_info);

  bool SmoothBoundaryLine(const LocalRoute& raw_local_route,
                          LocalRoute& local_route);

  void CalRoutePointHeading(std::vector<LocalRoutePoint>& xy_points);

  void GetBoundryAnchorPoints(
      const LocalRoute& local_route,
      std::vector<RouteAnchorPoint>* anchor_points) const;

  const bool IsLargeCurvatureSample() const;

  const double GetSampleInterval() const;

  void SetHostLCResult(const DEC_Outputs& fitted_result,
                       std::list<LocalRoute>& local_routes);

  void UpdateFitState();

 private:
  std::mutex pnc_map_mutex_;
  std::unique_ptr<hdmap::PncMap> pnc_map_;

  std::unique_ptr<MapPolynomialFit> polynomial_fitter_;
  std::list<std::pair<uint64_t, DEC_Outputs>> fitted_line_result_;
  std::mutex fitted_line_mutex_;

  std::mutex map_state_mutex_;  // map state mutex
  bool map_state_normal_ = false;

  std::mutex local_map_mutex_;  // local map mutex
  std::shared_ptr<zark::hdmap_new::ZarkMap> local_map_;

  std::mutex vehicle_state_mutex_;  // vehicle_state mutex
  common::VehicleState vehicle_state_;

  std::mutex local_routes_mutex_;
  bool is_local_route_updated_ = false;  // flag local route update succeed.
  std::queue<std::list<LocalRoute>> local_route_history_;
  std::queue<std::list<hdmap::RouteSegments>> route_segments_history_;

  std::mutex stop_flag_mutex_;
  bool is_stop_ = false;  // flag thread stop

  std::mutex is_olm_mutex_;
  bool is_olm_ = false;  // flag online map (Camera Perception map)

  double last_calculation_time_ = 0.0;
  uint64_t refline_start_time_ = 0;
  LocalRouteConfig local_route_config_;  // local route generate config

  std::unique_ptr<LocalRouteSmoother> smoother_;  // local route smoother
  std::unique_ptr<ACCTrajectoryBuilder> acc_trajectory_builder_;

  std::pair<FixNoaLineInfo, std::vector<LocalRoutePoint>> fix_noa_line_;
  std::unordered_map<std::string, FixNoaLineInfo> history_noa_lines_;
  zark::hdmap_new::Id adc_pre_lane_id_;
  BoundaryMoveInfo last_move_info_;
  std::string error_msg_;
};

}  // namespace planning
}  // namespace zark
