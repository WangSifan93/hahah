#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "apps/planning/src/common/vehicle_config.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/planning_msgs/scenario_type.h"
#include "apps/planning/src/planning_msgs/task_config.h"

using namespace zark::planning;

template <typename T>
class EnumParser {
 private:
  static std::map<std::string, T>& GetEnumMap();

 public:
  static T Parse(const std::string& value) {
    std::map<std::string, T>& enumMap = GetEnumMap();
    auto it = enumMap.find(value);
    if (it == enumMap.end())
      throw std::runtime_error("Invalid string value for enum");
    return it->second;
  }
};

template <typename T>
std::map<std::string, T>& EnumParser<T>::GetEnumMap() {
  static std::map<std::string, T>
      enumMap;  // Static to preserve the map across function calls
  return enumMap;
}

template <>
std::map<std::string, PlannerType>& EnumParser<PlannerType>::GetEnumMap() {
  static std::map<std::string, PlannerType> enumMap = {
      {"PUBLIC_ROAD", PlannerType::PUBLIC_ROAD}};
  return enumMap;
}

template <>
std::map<std::string, TaskConfig::TaskType>&
EnumParser<TaskConfig::TaskType>::GetEnumMap() {
  static std::map<std::string, TaskConfig::TaskType> enumMap = {
      {"TRAFFIC_RULE_DECIDER", TaskConfig::TaskType::TRAFFIC_RULE_DECIDER},
      {"MISSION_DECIDER", TaskConfig::TaskType::MISSION_DECIDER},
      {"LATERAL_DECIDER", TaskConfig::TaskType::LATERAL_DECIDER},
      {"LONGITUDINAL_DECIDER", TaskConfig::TaskType::LONGITUDINAL_DECIDER},
      {"LONGITUDINAL_OPTIMIZER", TaskConfig::TaskType::LONGITUDINAL_OPTIMIZER},
      {"LATERAL_OPTIMIZER", TaskConfig::TaskType::LATERAL_OPTIMIZER},
      {"EVALUATION_DECIDER", TaskConfig::TaskType::EVALUATION_DECIDER}};
  return enumMap;
}

template <>
std::map<std::string, ScenarioType>& EnumParser<ScenarioType>::GetEnumMap() {
  static std::map<std::string, ScenarioType> enumMap = {
      {"LANE_FOLLOW_V2", ScenarioType::LANE_FOLLOW_V2}};
  return enumMap;
}

template <>
std::map<std::string, StageType>& EnumParser<StageType>::GetEnumMap() {
  static std::map<std::string, StageType> enumMap = {
      {"NO_STAGE", StageType::NO_STAGE},
      {"LANE_FOLLOW_V2_DEFAULT_STAGE",
       StageType::LANE_FOLLOW_V2_DEFAULT_STAGE}};
  return enumMap;
}
