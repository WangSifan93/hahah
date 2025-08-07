/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file longitudinal_padding.h
 **/

#pragma once

#include "apps/planning/src/common/perception_obstacle.h"
#include "apps/planning/src/motion/longitudinal/data_type.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_lookup_tables.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

class LongitudinalPadding {
 public:
  LongitudinalPadding() = default;

  /**
   * @brief get front stiff padding.
   * @param lookup_tables longitudinal look tables
   * @param type obstacle type
   * @param is_virtual is virtual obstacle
   * @param v_obs obstacle velocity [m/s]
   * @param v_ego ego velocity [m/s]
   * @return front stiff padding [m]
   *
   */
  const double GetFrontStiffPadding(
      const LongitudinalLookupTables& lookup_tables,
      const perception::SubType& type, const bool is_virtual,
      const double v_obs, const double v_ego) const;

  /**
   * @brief get front soft padding.
   * @param lookup_tables longitudinal look tables
   * @param v_obs obstacle velocity [m/s]
   * @param v_ego ego velocity [m/s]
   * @return front soft padding [m]
   *
   */
  const double GetFrontSoftPadding(
      const LongitudinalLookupTables& lookup_tables, const double v_obs,
      const double v_ego) const;

  /**
   * @brief get rear stiff padding.
   * @param lookup_tables longitudinal look tables
   * @param type obstacle type
   * @param is_virtual is virtual obstacle
   * @param v_obs obstacle velocity [m/s]
   * @param v_ego ego velocity [m/s]
   * @return rear stiff padding [m]
   *
   */
  const double GetRearStiffPadding(
      const LongitudinalLookupTables& lookup_tables,
      const perception::SubType& type, const bool is_virtual,
      const double v_obs, const double v_ego) const;

  /**
   * @brief get rear soft padding.
   * @param lookup_tables longitudinal look tables
   * @param v_obs obstacle velocity [m/s]
   * @param v_ego ego velocity [m/s]
   * @return rear soft padding [m]
   *
   */
  const double GetRearSoftPadding(const LongitudinalLookupTables& lookup_tables,
                                  const double v_obs, const double v_ego) const;

  /**
   * @brief Get the Time Gap Multiplier object
   *
   * @param time_gap_level time gap level from state management
   * @return the relevant multiplier for following distance
   */
  double GetTimeGapMultiplier(const TimeGapLevel& time_gap_level) const;

 private:
  /**
   * @brief get stiff padding.
   * @param stiff_padding_table stiff padding table
   * @param type obstacle type
   * @param is_virtual is virtual obstacle
   * @return stiff padding [m]
   */
  const double GetStiffPadding(
      const LongitudinalOptimizerConfig::Constraint::StiffPaddingTable&
          stiff_padding_table,
      const perception::SubType& type, const bool is_virtual) const;

  /**
   * @brief get soft padding.
   * @param soft_padding_table soft padding table
   * @param v_obs obstacle velocity [m/s]
   * @return soft padding [m]
   */
  const double GetSoftPadding(const LookupTable& soft_padding_table,
                              const double v_obs) const;

  /**
   * @brief get padding multiplier.
   *
   * @param speed_tolerance_table speed tolerance table
   * @param relative_speed_multiplier_table  padding relative speed
   * multiplier table
   * @param v_front front object velocity [m/s]
   * @param v_rear rear object velocity [m/s]
   * @return padding multiplier
   */
  const double GetPaddingMultiplier(
      const LookupTable& speed_tolerance_table,
      const LookupTable& relative_speed_multiplier_table, const double v_front,
      const double v_rear) const;
};

}  // namespace planning
}  // namespace zark
