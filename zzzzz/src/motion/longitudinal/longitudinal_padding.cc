/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file longitudinal_padding.cc
 **/

#include "apps/planning/src/motion/longitudinal/longitudinal_padding.h"

namespace zark {
namespace planning {

const double LongitudinalPadding::GetFrontStiffPadding(
    const LongitudinalLookupTables& lookup_tables,
    const perception::SubType& type, const bool is_virtual, const double v_obs,
    const double v_ego) const {
  return GetStiffPadding(lookup_tables.front_stiff_padding_table(), type,
                         is_virtual) *
         GetPaddingMultiplier(
             lookup_tables.front_padding_speed_tolerance_table(),
             lookup_tables.front_padding_relative_speed_multiplier_table(),
             v_obs, v_ego);
}

const double LongitudinalPadding::GetFrontSoftPadding(
    const LongitudinalLookupTables& lookup_tables, const double v_obs,
    const double v_ego) const {
  return GetSoftPadding(lookup_tables.front_soft_padding_table(), v_obs) *
         GetPaddingMultiplier(
             lookup_tables.front_padding_speed_tolerance_table(),
             lookup_tables.front_padding_relative_speed_multiplier_table(),
             v_obs, v_ego);
}

const double LongitudinalPadding::GetRearStiffPadding(
    const LongitudinalLookupTables& lookup_tables,
    const perception::SubType& type, const bool is_virtual, const double v_obs,
    const double v_ego) const {
  return GetStiffPadding(lookup_tables.rear_stiff_padding_table(), type,
                         is_virtual) *
         GetPaddingMultiplier(
             lookup_tables.rear_padding_speed_tolerance_table(),
             lookup_tables.rear_padding_relative_speed_multiplier_table(),
             v_ego, v_obs);
}

const double LongitudinalPadding::GetRearSoftPadding(
    const LongitudinalLookupTables& lookup_tables, const double v_obs,
    const double v_ego) const {
  return GetSoftPadding(lookup_tables.rear_soft_padding_table(), v_obs) *
         GetPaddingMultiplier(
             lookup_tables.rear_padding_speed_tolerance_table(),
             lookup_tables.rear_padding_relative_speed_multiplier_table(),
             v_ego, v_obs);
}

const double LongitudinalPadding::GetStiffPadding(
    const LongitudinalOptimizerConfig::Constraint::StiffPaddingTable&
        stiff_padding_table,
    const perception::SubType& type, const bool is_virtual) const {
  if (is_virtual) {
    return stiff_padding_table.virtual_fence;
  }
  switch (type) {
    case perception::ST_BIG_BUS:
    case perception::ST_LIGHT_TRUCK:
    case perception::ST_HEAVY_TRUCK:
    case perception::ST_SMALL_BUS:
      return stiff_padding_table.large_car;
      break;
    case perception::ST_CAR:
      return stiff_padding_table.regular_car;
      break;
    case perception::ST_CYCLIST:
    case perception::ST_MOTORCYCLIST:
      return stiff_padding_table.bicycle;
      break;
    case perception::ST_PEDESTRIAN:
      return stiff_padding_table.pedestrian;
    default:
      return stiff_padding_table.others;
      break;
  }
}

const double LongitudinalPadding::GetSoftPadding(
    const LookupTable& soft_padding_table, const double v_obs) const {
  return soft_padding_table.Evaluate(v_obs);
}

const double LongitudinalPadding::GetPaddingMultiplier(
    const LookupTable& speed_tolerance_table,
    const LookupTable& relative_speed_multiplier_table, const double v_front,
    const double v_rear) const {
  const double v_tol = speed_tolerance_table.Evaluate(v_rear);
  return relative_speed_multiplier_table.Evaluate((v_front - v_rear) / v_tol);
}

double LongitudinalPadding::GetTimeGapMultiplier(
    const TimeGapLevel& time_gap_level) const {
  std::vector<double> kTimeGapLevelMultipliers{1.0, 0.67, 0.89,
                                               1.0, 1.11, 1.33};
  return kTimeGapLevelMultipliers[static_cast<int>(time_gap_level)];
}

}  // namespace planning
}  // namespace zark
