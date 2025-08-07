#include "apps/planning/src/reference_line_provider/pnc_map/hdmap_api.h"

namespace zark {
namespace planning {
namespace hdmap {

std::shared_ptr<zmap::MapService> HDMapUtil::map_service_ptr_ =
    std::make_shared<zmap::MapService>("planning");

std::shared_ptr<zmap::ZpilotMapInfo> HDMapUtil::hdmap_ptr_ =
    std::make_shared<zmap::ZpilotMapInfo>();

std::mutex HDMapUtil::map_service_mutex_;

std::mutex HDMapUtil::base_map_mutex_;

const std::shared_ptr<zmap::ZpilotMapInfo> HDMapUtil::BaseMapPtr() {
  std::lock_guard<std::mutex> lock(base_map_mutex_);
  return hdmap_ptr_;
}

void HDMapUtil::SetBaseMapPtr(
    const std::shared_ptr<zmap::ZpilotMapInfo> &map_ptr) {
  std::lock_guard<std::mutex> lock(base_map_mutex_);
  hdmap_ptr_ = map_ptr;
}

const std::shared_ptr<zmap::MapService> HDMapUtil::MapServicePtr(
    const bool need_lock) {
  std::unique_lock<std::mutex> lock(map_service_mutex_, std::defer_lock);
  if (need_lock) {
    lock.lock();
  }
  return map_service_ptr_;
}

}  // namespace hdmap
}  // namespace planning
}  // namespace zark
