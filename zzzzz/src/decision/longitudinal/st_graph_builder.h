/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file: st_graph_builder.h
 **/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "apps/planning/src/common/corridor.h"
#include "box2d.h"
#include "vec2d.h"
#include "status.h"
#include "apps/planning/src/common/obstacle.h"
#include "apps/planning/src/common/speed/st_boundary.h"
#include "apps/planning/src/common/vehicle_config.h"
#include "apps/planning/src/decision/longitudinal/data_type.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

/**
 * class STGraphBuilder
 * @brief Build ST graph with corridor and obstacles.
 */
class STGraphBuilder {
 public:
  STGraphBuilder(const LongitudinalDeciderConfig::STGraphConfig& config);

  /**
   * @brief Build ST graph with corridor and obstacles.
   *
   * @param corridor A list of corridor points.
   * @param obstacles A list of obstacles.
   * @param obs_sl_boundary_map A map of SL boundaries of all obstacles.
   * @param st_graph ST graph built.
   * @param lateral_obstacles Object list which inclue lest obstacles and
   * right obstacles.
   * @param st_boundary_counter_map ST Boundary counter map.
   * @return Status Status of st graph builder.
   */
  ::common::Status BuildSTGraph(
      const Corridor& corridor,
      const IndexedPtrList<std::string, const Obstacle*>& obstacle_map,
      const std::unordered_map<const Obstacle*, SLTrajectory>&
          obs_sl_boundary_map,
      std::vector<STBoundary>& st_graph,
      std::vector<std::pair<const Obstacle*, bool>>& lateral_obstacles,
      std::unordered_map<std::string, int>& st_boundary_counter_map);

 private:
  /**
   * @brief Map obstacles to ST boundarys with corridor and determine left
   * obstacles and right obstacles.
   *
   * @param corridor A list of corridor points.
   * @param obstacles A list of obstacles.
   * @param obs_sl_boundary_map A map of SL boundaries of all obstacles.
   * @param st_graph ST graph built.
   * @param lateral_obstacles Object list which inclue lest obstacles and
   * right obstacles.
   * @param st_boundary_counter_map ST Boundary counter map.
   * @return Status Status of mapping obstacles to ST boundaries.
   */
  ::common::Status MapObstaclesToSTBoundaries(
      const Corridor& corridor,
      const IndexedPtrList<std::string, const Obstacle*>& obstacle_map,
      const std::unordered_map<const Obstacle*, SLTrajectory>&
          obs_sl_boundary_map,
      std::vector<STBoundary>& st_graph,
      std::vector<std::pair<const Obstacle*, bool>>& lateral_obstacles,
      std::unordered_map<std::string, int>& st_boundary_counter_map);

  /**
   * @brief Update st boundary counter map by obstacle.
   *
   * @param obstacle_map A list of obstacles.
   * @param st_boundary_counter_map ST Boundary counter map.
   */
  void UpdateSTBoundaryCounterMap(
      const IndexedPtrList<std::string, const Obstacle*>& obstacle_map,
      std::unordered_map<std::string, int>& st_boundary_counter_map);

  /**
   * @brief Given a single obstacle, compute its ST-boundary by its overlapping.
   *
   * @param obstacle An obstacle.
   * @param adc_path_points A vector of ADC planned path points.
   * @param lower_points A vector to be filled with lower edge of ST-polygon.
   * @param upper_points A vector to be filled with upper edge of ST-polygon.
   * @param is_lateral Is a lateral obstacle.
   * @param is_left Is a left lateral obstacle.
   * @return bool If appears on ST-graph, return true; otherwise, false.
   */

  bool ComputeObstacleSTBoundaryByObsOverlap(const Obstacle& obstacle,
                                             const Corridor& adc_path_points,
                                             const SLTrajectory& sl_boudaries,
                                             std::vector<STPoint>& lower_points,
                                             std::vector<STPoint>& upper_points,
                                             bool& is_lateral, bool& is_left);
  /**
   * @brief Given a single obstacle, compute its ST-boundary by its SL-boundary.
   *
   * @param obstacle An obstacle.
   * @param adc_path_points A vector of ADC planned path points.
   * @param lower_points A vector to be filled with lower edge of ST-polygon.
   * @param upper_points A vector to be filled with upper edge of ST-polygon.
   * @param is_lateral Is a lateral obstacle.
   * @param is_left Is a left lateral obstacle.
   * @return bool If appears on ST-graph, return true; otherwise, false.
   */
  bool ComputeObstacleSTBoundaryByObsSL(const Obstacle& obstacle,
                                        const Corridor& adc_path_points,
                                        const SLTrajectory& sl_boudaries,
                                        std::vector<STPoint>& lower_points,
                                        std::vector<STPoint>& upper_points,
                                        bool& is_lateral, bool& is_left);

  /**
   * @brief Given ADC's path and an obstacle instance at a certain
   * timestep, get the upper and lower s that ADC might overlap with the obs
   * instance.
   *
   * @param adc_path_points A vector of ADC planned path points.
   * @param obstacle_instance A obstacle at a certain timestep.
   * @param adc_l_buffer Lateral buffer for safety consideration.
   * @param overlapping_s The overlapping upper and lower is to be updated.
   * @return bool Whether there is an overlap or not.
   */
  bool GetOverlappingS(const Corridor& adc_path_points,
                       const ::math::Box2d& obstacle_instance,
                       const double adc_l_buffer,
                       std::pair<double, double>* const overlapping_s);

  /**
   * @brief Over the s-dimension, find the last point that is before the
   * obstacle instance of the first point that is after the obstacle.
   * If there exists no such point within the given range, return -1.
   *
   * @param adc_path_points A vector of ADC planned path points.
   * @param obstacle_instance A obstacle at a certain timestep.
   * @param s_thresh The s threshold, must be non-negative.
   * @param is_before The direction.
   * @param start_idx The start-idx.
   * @param end_idx The end-idx.
   * @return int Whether there is overlapping or not.
   */
  int GetSBoundingPathPointIndex(const Corridor& adc_path_points,
                                 const ::math::Box2d& obstacle_instance,
                                 const double s_thresh, const bool is_before,
                                 const int start_idx, const int end_idx);

  /**
   * @brief Over the s-dimension, check if the path-point is away
   * from the projected obstacle in the given direction.
   *
   * @param path_point A certain path-point.
   * @param direction_point The next path-point indicating path direction.
   * @param obs_box The obstacle bounding box.
   * @param s_thresh The threshold s to tell if path point is far away.
   * @param is_before Direction indicator. True if we want the path-point to be
   *        before the obstacle.
   * @return bool whether the path-point is away in the indicated direction.
   */
  bool IsPathPointAwayFromObstacle(const CorridorPoint& path_point,
                                   const CorridorPoint& direction_point,
                                   const ::math::Box2d& obs_box,
                                   const double s_thresh, const bool is_before);

  /**
   * @brief Check if ADC is overlapping with the given obstacle box.
   *
   * @param adc_path_point ADC's position.
   * @param obs_box Obstacle's box.
   * @param l_buffer ADC's lateral buffer.
   * @return bool Whether ADC at that position is overlapping with the given
   * obstacle box.
   */
  bool IsADCOverlappingWithObstacle(const CorridorPoint& adc_path_point,
                                    const ::math::Box2d& obs_box,
                                    const double l_buffer);

  /**
   * @brief Check if obstacle is close to corridor and determine
   * whether it is left or right obstacle.
   *
   * @param adc_path_points A vector of ADC planned path points.
   * @param sl_boundary Obstacle's sl boundary.
   * @param is_left_obstacle Obstacle is at left of ego vehicle.
   * @param is_right_obstacle Obstacle is at right of ego vehicle.
   */
  void DetermineLeftOrRightObstacle(const Corridor& adc_path_points,
                                    const SLBoundary sl_boundary,
                                    bool& is_lateral, bool& is_left);

  /**
   * @brief Get left and right lateral position of corridor with an given
   * obstacle_s.
   *
   * @param adc_path_points A vector of ADC planned path points.
   * @param obstacle_s Obstacle's position s.
   * @param is_left Indicator to calculate left lateral position, if false,
   * calculate right lateral position.
   * @param lateral Left or right lateral position calculated of corridor.
   * @return bool Whether interpolation by s to get l is success.
   */
  bool GetLeftAndRightLbyS(const Corridor& adc_path_points,
                           const double obstacle_s, const bool is_left,
                           double* lateral);

  /**
   * @brief Determine whether obstacle is fully-bloking with corridor.
   *
   * @param corridor Corridor points.
   * @param s Position s in corridor of obstacle [m].
   * @param l_start Start l of obstacle [m].
   * @param l_end End l of obstacle [m].
   * @return bool Whether obstacle is fully-blocking.
   */
  bool IsFullyBlocking(const Corridor& corridor, const double s,
                       const double l_start, const double l_end);

 private:
  LongitudinalDeciderConfig::STGraphConfig config_;
  FRIEND_TEST(STGraphBuilderTest, TestMapObstaclesToSTBoundaries);
  FRIEND_TEST(STGraphBuilderTest, TestMapObstaclesToSTBoundariesPlanB);
  FRIEND_TEST(STGraphBuilderTest, TestComputeObstacleSTBoundaryByObsOverlap);
  FRIEND_TEST(STGraphBuilderTest, TestComputeObstacleSTBoundaryByObsSL);
  FRIEND_TEST(STGraphBuilderTest, TestGetOverlappingS);
  FRIEND_TEST(STGraphBuilderTest, TestGetSBoundingPathPointIndex);
  FRIEND_TEST(STGraphBuilderTest, TestIsPathPointAwayFromObstacle);
  FRIEND_TEST(STGraphBuilderTest, TestIsADCOverlappingWithObstacle);
  FRIEND_TEST(STGraphBuilderTest, TestDetermineLeftOrRightObstacle);
  FRIEND_TEST(STGraphBuilderTest, TestGetLeftAndRightLbyS);
  FRIEND_TEST(STGraphBuilderTest, TestIsFullyBlocking);
  FRIEND_TEST(STGraphBuilderTest, TestUpdateSTBoundaryCounterMap);
};

}  // namespace planning
}  // namespace zark
