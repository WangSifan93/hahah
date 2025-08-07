
#ifndef _APPS_E2EPNC_SRC_E2EPNC_COMPONENT_H_
#define _APPS_E2EPNC_SRC_E2EPNC_COMPONENT_H_
#include "common/input_frame.h"
#include "common/planning_macros.h"
#include "common/type_def.h"
#include "dataflow/callback/callback.h"
#include "dataflow/module/module.h"
#include "dataflow/module/module_option.h"
#include "dataflow/module/proc.h"
#include "maps/map_container.h"
#include "maps/map_def.h"
#include "nodes/e2e_planning_core.h"
#include "nodes/subscriber.h"

class E2EPncComponent : public hobot::dataflow::Module {
 public:
  E2EPncComponent(const hobot::dataflow::ModuleOption &module_option);
  ~E2EPncComponent();
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
  void VehicleStatusCallback(hobot::dataflow::spMsgResourceProc proc,
                             const hobot::dataflow::MessageLists &msgs);
  void FctOutputsCallback(hobot::dataflow::spMsgResourceProc proc,
                          const hobot::dataflow::MessageLists &msgs);

  void E2EPncProc(hobot::dataflow::spMsgResourceProc proc,
                  const hobot::dataflow::MessageLists &msgs);
  void E2EPncDebugProc(hobot::dataflow::spMsgResourceProc proc,
                       const hobot::dataflow::MessageLists &msgs);

  void VehChassis_DataProc(hobot::dataflow::spMsgResourceProc proc,
                           const hobot::dataflow::MessageLists &msgs);

  std::pair<ad_e2e::planning::PlanResult,
            std::shared_ptr<ad_e2e::planning::PlanningInputFrame>>
  GetPlanningInputFrame();

 protected:
  int32_t Init() override;
  static constexpr size_t kPredictionCounter = 0;
  static constexpr size_t kPlanningCounter = 1;

 private:
  std::shared_ptr<hobot::time::ClockWrapper> clock_wrapper_;
  std::shared_ptr<hobot::time::Clock> clock_;

  ad_e2e::planning::MapInfo CreateMapInfo(
      const zark::MapMsgType &map_msg, const std::string &ego_road_section_id,
      bool is_on_highway = false);
  zark::prediction::proto::PredictionObjects prediction_obstacles_;
  zark::localization::LocalizationInfo localization_estimate_;
  zark::mapfusion::MapLocalizationInfo map_localization_;
  zark::zdrive_map::ZarkMap map_;

  ad_e2e::planning::SingleSubscriber<zark::MapMsgType> map_msg_;
  ad_e2e::planning::SingleSubscriber<zark::PredictionObjs> predic_objs_msg_;
  ad_e2e::planning::SingleSubscriber<zark::MapFusionMsgType> map_fusion_msg_;

  ad_e2e::planning::QueueSubscriber<zark::OdometryMsgType> localization_msg_;
  ad_e2e::planning::QueueSubscriber<zark::VehicleStatusType> veh_status_msg_;
  ad_e2e::planning::QueueSubscriber<zark::FctMsgType> fct_msg_;

  ad_e2e::planning::MapContainer map_container_;
  std::string ego_road_section_id_ = "";

  int64_t planning_callback_start_ms_{0};
  pthread_t planning_callback_tid_{0};
  std::mutex thread_info_mutex_;
  std::mutex debug_info_lock_;

  ad_e2e::planning::E2EPlanningCore::UPtr core_;

  uint32_t sequence_num_ = 0;
};

#endif
