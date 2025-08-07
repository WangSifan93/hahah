
#pragma once
#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "signal_base.h"
#include "signal_config.h"
#include "signal_factory.h"

namespace e2e_noa {
namespace planning {

// 扩展工厂功能 - 批量创建和管理
class SignalManager {
 public:
  // 从配置列表批量创建信号
  static std::vector<std::unique_ptr<Signal>> CreateSignalsFromConfigs(
      const std::vector<SignalConfig>& configs) {
    std::vector<std::unique_ptr<Signal>> signals;
    signals.reserve(configs.size());

    for (const auto& config : configs) {
      auto signal = SignalFactory::CreateSignal(config);
      if (signal) {
        signals.push_back(std::move(signal));
      }
    }

    return signals;
  }

  // 按类型分组创建信号
  static absl::flat_hash_map<SignalType, std::vector<std::unique_ptr<Signal>>>
  CreateSignalsByType(const std::vector<SignalConfig>& configs) {
    absl::flat_hash_map<SignalType, std::vector<std::unique_ptr<Signal>>>
        grouped_signals;

    for (const auto& config : configs) {
      auto signal = SignalFactory::CreateSignal(config);
      if (signal) {
        grouped_signals[config.type].push_back(std::move(signal));
      }
    }

    return grouped_signals;
  }

  // 创建用于BehaviorDescriptor的信号映射
  static absl::flat_hash_map<SignalType, std::unique_ptr<Signal>>
  CreateSignalMap(const std::vector<SignalConfig>& configs) {
    absl::flat_hash_map<SignalType, std::unique_ptr<Signal>> signal_map;

    for (const auto& config : configs) {
      auto signal = SignalFactory::CreateSignal(config);
      if (signal) {
        // 如果同类型已存在，替换为最新的
        signal_map[config.type] = std::move(signal);
      }
    }

    return signal_map;
  }
};

}  // namespace planning
}  // namespace e2e_noa