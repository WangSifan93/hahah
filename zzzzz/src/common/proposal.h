/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file proposal.h
 **/

#pragma once

#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/common/trajectory/discretized_trajectory.h"
#include "apps/planning/src/decision/longitudinal/data_type.h"

namespace zark {
namespace planning {

using Costs = std::vector<std::pair<double, std::string>>;  // <cost, reason>

class Proposal {
 public:
  Proposal() = default;
  Proposal(const Proposal& proposal) = default;

  explicit Proposal(const CorridorInfo* const corridor_info,
                    const STProposal& st_proposal,
                    const LonMPCData& lon_mpc_data,
                    const LatMPCData& lat_mpc_data,
                    const DiscretizedTrajectory& trajectory,
                    const std::unordered_map<std::string, Costs>& costs);

  inline const CorridorInfo* const GetCorridorInfo() const {
    return corridor_info_;
  }

  inline const STProposal& GetSTProposal() const { return st_proposal_; }

  inline const LonMPCData& GetLonMPCData() const { return lon_mpc_data_; }

  inline const LatMPCData& GetLatMPCData() const { return lat_mpc_data_; }

  inline const DiscretizedTrajectory& GetTrajectory() const {
    return trajectory_;
  }

  inline const std::unordered_map<std::string, Costs>& GetCosts() const {
    return costs_;
  }

  std::unordered_map<std::string, Costs>* GetMutableCosts() { return &costs_; }

  inline void SetTrajectory(const DiscretizedTrajectory& trajectory) {
    trajectory_ = trajectory;
  }

  void SetLonMPCData(const LonMPCData& lon_mpc_data) {
    lon_mpc_data_ = lon_mpc_data;
  }

  void SetLatMPCData(const LatMPCData& lat_mpc_data) {
    lat_mpc_data_ = lat_mpc_data;
  }

  void SetCorridorInfo(const CorridorInfo* corridor_info) {
    corridor_info_ = corridor_info;
  }

  void SetSTProposal(const STProposal& st_proposal) {
    st_proposal_ = st_proposal;
  }

 private:
  const CorridorInfo* corridor_info_{nullptr};
  STProposal st_proposal_;
  LonMPCData lon_mpc_data_;
  LatMPCData lat_mpc_data_;
  DiscretizedTrajectory trajectory_;
  std::unordered_map<std::string, Costs> costs_;  // cost map
};

}  // namespace planning
}  // namespace zark
