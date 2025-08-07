// Copyright (c) [2021-2022] [Horizon Robotics][Horizon Bole].
//
// You can use this software according to the terms and conditions of
// the Apache v2.0.
// You may obtain a copy of Apache v2.0. at:
//
//     http: //www.apache.org/licenses/LICENSE-2.0
//
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See Apache v2.0 for more details.
#ifndef _APPS_PLANNING_SRC_PLANNING_COMPONENT_H_
#define _APPS_PLANNING_SRC_PLANNING_COMPONENT_H_
#include "apps/planning/src/planning_base.h"
#include "dataflow/callback/callback.h"
#include "dataflow/module/module.h"
#include "dataflow/module/module_option.h"
#include "dataflow/module/proc.h"

class PlanningComponent : public hobot::dataflow::Module {
 public:
  PlanningComponent(const hobot::dataflow::ModuleOption &module_option);
  ~PlanningComponent();
  void InitPortsAndProcs() override;
  int32_t Start() override;
  int32_t Stop() override;
  void Reset() override;
  int32_t DeInit() override;
  void PredictionCallback(hobot::dataflow::spMsgResourceProc proc,
                          const hobot::dataflow::MessageLists &msgs);
  void LocalizationCallback(hobot::dataflow::spMsgResourceProc proc,
                            const hobot::dataflow::MessageLists &msgs);
  void MapCallback(hobot::dataflow::spMsgResourceProc proc,
                   const hobot::dataflow::MessageLists &msgs);
  void MapLocalizationCallback(hobot::dataflow::spMsgResourceProc proc,
                               const hobot::dataflow::MessageLists &msgs);
  void ChassisCallback(hobot::dataflow::spMsgResourceProc proc,
                       const hobot::dataflow::MessageLists &msgs);
  void CanDataCallback(hobot::dataflow::spMsgResourceProc proc,
                       const hobot::dataflow::MessageLists &msgs);
  void FctDataCallback(hobot::dataflow::spMsgResourceProc proc,
                       const hobot::dataflow::MessageLists &msgs);
  void PlanningProc(hobot::dataflow::spMsgResourceProc proc,
                    const hobot::dataflow::MessageLists &msgs);
  void PlanningRefLineProc(hobot::dataflow::spMsgResourceProc proc,
                           const hobot::dataflow::MessageLists &msgs);
  void PlanningDebugProc(hobot::dataflow::spMsgResourceProc proc,
                         const hobot::dataflow::MessageLists &msgs);
  void HeartbeatProc(hobot::dataflow::spMsgResourceProc proc,
                     const hobot::dataflow::MessageLists &msgs);
  void NoaDecisionMeasureProc(hobot::dataflow::spMsgResourceProc proc,
                              const hobot::dataflow::MessageLists &msgs);

 protected:
  int32_t Init() override;

 private:
  void InitLocalView();
  void SendNoaDecisionResult(const hobot::dataflow::spMsgResourceProc &proc);

 private:
  zark::prediction::proto::PredictionObjects prediction_obstacles_;
  zark::localization::LocalizationInfo localization_estimate_;
  zark::mapfusion::MapLocalizationInfo map_localization_;
  zark::ads_fct::FCT_Outputs fct_output_;
  zark::ads_adptrin::ADAS_Inputs can_data_;
  zark::hdmap_new::ZarkMap all_map_;
  zark::planning::LocalView local_view_;
  std::mutex local_view_lock_;

  std::unique_ptr<zark::planning::PlanningBase> planning_base_;
  std::shared_ptr<zark::planning::DependencyInjector> injector_;
  zark::planning::PlanningConfig config_;
  zark::planning::PlanningDebugInfo debug_info_;
  std::mutex debug_info_lock_;
  zark::reference_line_proto::ReferenceLines ref_lines_;
  std::mutex path_ref_line_lock_;
  uint32_t sequence_num_ = 0;
};

#endif /* _APPS_PLANNING_SRC_PLANNING_COMPONENT_H_ */
