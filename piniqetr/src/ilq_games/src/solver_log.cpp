//////////////////////////////////////////////////////////////////////////////
//
// Container to store solver logs.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/utils/solver_log.h"

#include <glog/logging.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <regex>
#include <vector>

#include "common/log_data.h"
#include "ilq_games/include/examples/muti_vehicle_game.h"
#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/operating_point.h"
#include "ilq_games/include/utils/strategy.h"
#include "nlohmann/nlohmann/json.hpp"

using json = nlohmann::json;

namespace e2e_noa::planning {

VectorXf SolverLog::InterpolateState(size_t iterate, Time t) const {
  const OperatingPoint &op = operating_points_[iterate];

  // Low and high indices between which to interpolate.
  const size_t lo = TimeToIndex(t);
  const size_t hi = std::min(lo + 1, op.xs.size() - 1);

  // Fraction of the way between lo and hi.
  const float frac = (t - IndexToTime(lo)) / time::getTimeStep();
  return (1.0 - frac) * op.xs[lo] + frac * op.xs[hi];
}

float SolverLog::InterpolateState(size_t iterate, Time t, Dimension dim) const {
  const OperatingPoint &op = operating_points_[iterate];

  // Low and high indices between which to interpolate.
  const size_t lo = TimeToIndex(t);
  const size_t hi = std::min(lo + 1, op.xs.size() - 1);

  // Fraction of the way between lo and hi.
  const float frac = (t - IndexToTime(lo)) / time::getTimeStep();
  return (1.0 - frac) * op.xs[lo](dim) + frac * op.xs[hi](dim);
}

VectorXf SolverLog::InterpolateControl(size_t iterate, Time t,
                                       PlayerIndex player) const {
  const OperatingPoint &op = operating_points_[iterate];

  // Low and high indices between which to interpolate.
  const size_t lo = TimeToIndex(t);
  const size_t hi = std::min(lo + 1, op.xs.size() - 1);

  // Fraction of the way between lo and hi.
  const float frac = (t - IndexToTime(lo)) / time::getTimeStep();
  return (1.0 - frac) * op.us[lo][player] + frac * op.us[hi][player];
}

float SolverLog::InterpolateControl(size_t iterate, Time t, PlayerIndex player,
                                    Dimension dim) const {
  const OperatingPoint &op = operating_points_[iterate];

  // Low and high indices between which to interpolate.
  const size_t lo = TimeToIndex(t);
  const size_t hi = std::min(lo + 1, op.xs.size() - 1);

  // Fraction of the way between lo and hi.
  const float frac = (t - IndexToTime(lo)) / time::getTimeStep();
  return (1.0 - frac) * op.us[lo][player](dim) + frac * op.us[hi][player](dim);
}

bool SolverLog::Save(
    const std::vector<VehicleStaticData> &vehicles_static_dataset,
    const std::vector<VehicleDynamicData> &vehicles_dynamic_dataset,
    const std::string &experiment_name, const uint64_t seq_num,
    const bool enable_online_log, const bool enable_offline_log) const {
  // Making top-level directory
  const size_t num_vehicles = vehicles_static_dataset.size();

  size_t start = 0;

  const auto &op = operating_points_[0];
  json j;  // 创建 JSON 对象

  int timestep = 0;
  for (const auto &x : op.xs) {
    for (size_t veh = 0; veh < num_vehicles; ++veh) {
      const auto &vehicle_static_data = vehicles_static_dataset[veh];
      const auto &vehicle_dynamic_data = vehicles_dynamic_dataset[veh];
      std::string vehicle_id =
          "vehicle_" + std::to_string(vehicle_static_data.id);
      if (timestep == 0) {
        j["vehicles"][vehicle_id]["desired_v"] = vehicle_dynamic_data.tar_v;
        j["vehicles"][vehicle_id]["type"] = vehicle_static_data.type;
        j["vehicles"][vehicle_id]["width"] = vehicle_static_data.width;
        j["vehicles"][vehicle_id]["length"] = vehicle_static_data.length;
        for (const auto &point : vehicle_dynamic_data.lane) {
          j["vehicles"][vehicle_id]["ref_line"].push_back(
              {point.x(), point.y()});
        }
      }

      json traj;
      traj["time"] = timestep * time::getTimeStep();
      traj["x"] = x[6 * veh + 0];
      traj["y"] = x[6 * veh + 1];
      traj["heading"] = x[6 * veh + 2];
      traj["wheel_angle"] = x[6 * veh + 3];
      traj["v"] = x[6 * veh + 4];
      traj["a"] = x[6 * veh + 5];
      j["vehicles"][vehicle_id]["traj"][std::to_string(timestep)] = traj;
    }
    ++timestep;
  }

  // 写入文件
  if (enable_offline_log) {
    std::string directory = "./debug_ilq";
    if (!std::filesystem::exists(directory)) {
      if (!std::filesystem::create_directories(directory)) {
        return false;
      }
    }
    const std::string dir_name = directory;
    LOG(INFO) << "Saving to directory: " << dir_name;
    std::filesystem::path json_path =
        std::filesystem::path(directory) /
        (std::to_string(seq_num) + "_vehicle_traj.json");
    std::ofstream file(json_path);
    if (!file) {
      LOG(INFO) << "Failed to open file: " << json_path;
      return false;
    }
    file << j.dump(4);
    file.close();
  }
  if (enable_online_log) {
    std::string json_str = j.dump(4);
    Log2FG::LogDataV0("ILQGAME", json_str);
  }

  return true;
}

inline std::vector<MatrixXf> SolverLog::Ps(size_t iterate,
                                           size_t time_index) const {
  std::vector<MatrixXf> Ps(strategies_[iterate].size());
  for (PlayerIndex ii = 0; ii < Ps.size(); ii++)
    Ps[ii] = P(iterate, time_index, ii);
  return Ps;
}

inline std::vector<VectorXf> SolverLog::alphas(size_t iterate,
                                               size_t time_index) const {
  std::vector<VectorXf> alphas(strategies_[iterate].size());
  for (PlayerIndex ii = 0; ii < alphas.size(); ii++)
    alphas[ii] = alpha(iterate, time_index, ii);
  return alphas;
}

inline MatrixXf SolverLog::P(size_t iterate, size_t time_index,
                             PlayerIndex player) const {
  return strategies_[iterate][player].Ps[time_index];
}

inline VectorXf SolverLog::alpha(size_t iterate, size_t time_index,
                                 PlayerIndex player) const {
  return strategies_[iterate][player].alphas[time_index];
}

std::string DefaultExperimentName() {
  const auto date =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string name = std::string(std::ctime(&date));
  std::transform(name.begin(), name.end(), name.begin(),
                 [](char ch) { return (ch == ' ' || ch == ':') ? '_' : ch; });
  return std::regex_replace(name, std::regex("( |\n)+$"), "");
}
}  // namespace e2e_noa::planning
