//
// Copyright 2024 ZDrive.AI. All Rights Reserved.
//

#pragma once

#include <algorithm>
#include <limits>
#include <unordered_map>

#include "apps/map_service/map_info/map_service.h"
#include "apps/planning/src/common/log.h"

namespace zark {
namespace planning {
namespace hdmap {

constexpr double kCentimeterToMetre = 0.01;
using namespace zark::map_service;

/**
 * @class HDMapUtil
 * @brief The class of HDMapUtil.
 *        It provides hdmap_service pointer for  planning.
 */
class HDMapUtil {
 public:
  // Get MapService ptr
  static const std::shared_ptr<zmap::MapService> MapServicePtr(
      const bool need_lock = true);
  static const std::shared_ptr<zmap::ZpilotMapInfo> BaseMapPtr();
  static void SetBaseMapPtr(
      const std::shared_ptr<zmap::ZpilotMapInfo> &map_ptr);

 public:
  static std::mutex map_service_mutex_;

 private:
  HDMapUtil() = delete;
  static std::shared_ptr<zmap::MapService> map_service_ptr_;
  static std::shared_ptr<zmap::ZpilotMapInfo> hdmap_ptr_;
  static std::mutex base_map_mutex_;
};

/**
 * @brief create a Map ID given a string.
 * @param id a string id
 * @return a Map ID instance
 */
inline const hdmap_new::Id MakeMapId(const std::string &id) {
  zark::hdmap_new::Id map_id;
  if (id.empty()) {
    map_id.set_id("-1");
    AWARN << "id is empty.";
    return map_id;
  }
  if (id.length() == 0) {
    map_id.set_id("-1");
    AWARN << "id is empty.";
    return map_id;
  }
  map_id.set_id(id);
  return map_id;
}

/**
 * @brief get a Map lane given a id.
 * @param new_hdmap hdmap_service ptr
 * @param id a map id
 * @return a LaneInfoConstPtr
 */
inline const zmap::LaneInfoConstPtr GetLaneById(
    const std::shared_ptr<zmap::ZpilotMapInfo> new_hdmap,
    const zark::hdmap_new::Id &id, const bool need_lock = true) {
  zmap::MapFeaturePtr map_info;
  zmap::LaneInfoConstPtr lane_info = nullptr;
  if (new_hdmap == nullptr) return lane_info;
  std::unique_lock<std::mutex> lock(HDMapUtil::map_service_mutex_,
                                    std::defer_lock);
  if (need_lock) lock.lock();
  const zmap::MapFeatureType lb_type = zmap::MapFeatureType::kLaneInfo;
  if (new_hdmap->GetFeatureById(id, lb_type, &map_info)) {
    lane_info = std::dynamic_pointer_cast<const zmap::LaneInfo>(map_info);
  }
  return lane_info;
}

/**
 * @brief get a Map Road given a id.
 * @param new_hdmap hdmap_service ptr
 * @param id a map id
 * @return RoadInfoConstPtr
 */
inline const zmap::RoadInfoConstPtr GetRoadById(
    const std::shared_ptr<zmap::ZpilotMapInfo> new_hdmap,
    const zark::hdmap_new::Id &id, const bool need_lock = true) {
  zmap::MapFeaturePtr map_info;
  zmap::RoadInfoConstPtr road_info = nullptr;
  if (new_hdmap == nullptr) return road_info;
  std::unique_lock<std::mutex> lock(HDMapUtil::map_service_mutex_,
                                    std::defer_lock);
  if (need_lock) lock.lock();
  const zmap::MapFeatureType lb_type = zmap::MapFeatureType::kRoadInfo;
  if (new_hdmap->GetFeatureById(id, lb_type, &map_info)) {
    road_info = std::dynamic_pointer_cast<const zmap::RoadInfo>(map_info);
  }
  return road_info;
}

/**
 * @brief get a Map RoadSection given a id.
 * @param new_hdmap hdmap_service ptr
 * @param id a map id
 * @return RoadSectionInfoConstPtr
 */
inline const zmap::RoadSectionInfoConstPtr GetRoadSectionById(
    const std::shared_ptr<zmap::ZpilotMapInfo> new_hdmap,
    const zark::hdmap_new::Id &id, const bool need_lock = true) {
  zmap::MapFeaturePtr map_info;
  zmap::RoadSectionInfoConstPtr road_section_info = nullptr;
  if (new_hdmap == nullptr) return road_section_info;
  std::unique_lock<std::mutex> lock(HDMapUtil::map_service_mutex_,
                                    std::defer_lock);
  if (need_lock) lock.lock();
  const zmap::MapFeatureType lb_type = zmap::MapFeatureType::kRoadSectionInfo;
  if (new_hdmap->GetFeatureById(id, lb_type, &map_info)) {
    road_section_info =
        std::dynamic_pointer_cast<const zmap::RoadSectionInfo>(map_info);
  }
  return road_section_info;
}

/**
 * @brief get a Map RoadBoundary given a id.
 * @param new_hdmap hdmap_service ptr
 * @param id a map id
 * @return RoadBoundaryInfoConstPtr
 */
inline const zmap::RoadBoundaryInfoConstPtr GetRoadBoundaryById(
    const std::shared_ptr<zmap::ZpilotMapInfo> new_hdmap,
    const zark::hdmap_new::Id &id, const bool need_lock = true) {
  zmap::MapFeaturePtr map_info;
  zmap::RoadBoundaryInfoConstPtr road_boundary_info = nullptr;
  if (new_hdmap == nullptr) return road_boundary_info;
  std::unique_lock<std::mutex> lock(HDMapUtil::map_service_mutex_,
                                    std::defer_lock);
  if (need_lock) lock.lock();
  const zmap::MapFeatureType lb_type = zmap::MapFeatureType::kRoadBoundaryInfo;
  if (new_hdmap->GetFeatureById(id, lb_type, &map_info)) {
    road_boundary_info =
        std::dynamic_pointer_cast<const zmap::RoadBoundaryInfo>(map_info);
  }
  return road_boundary_info;
}

/**
 * @brief get TrafficSign given a id.
 * @param new_hdmap hdmap_service ptr
 * @param id a map id
 * @return TrafficSignInfoConstPtr
 */
inline const zmap::TrafficSignInfoConstPtr GetTrafficSignById(
    const std::shared_ptr<zmap::ZpilotMapInfo> new_hdmap,
    const zark::hdmap_new::Id &id, const bool need_lock = true) {
  zmap::MapFeaturePtr map_info;
  zmap::TrafficSignInfoConstPtr sign_info = nullptr;
  if (new_hdmap == nullptr) return sign_info;
  std::unique_lock<std::mutex> lock(HDMapUtil::map_service_mutex_,
                                    std::defer_lock);
  if (need_lock) lock.lock();
  const zmap::MapFeatureType lb_type = zmap::MapFeatureType::kTrafficSignInfo;
  if (new_hdmap->GetFeatureById(id, lb_type, &map_info)) {
    sign_info =
        std::dynamic_pointer_cast<const zmap::TrafficSignInfo>(map_info);
  }
  return sign_info;
}

/**
 * @brief get LaneBoundary given a id.
 * @param new_hdmap hdmap_service ptr
 * @param id a map id
 * @return LaneBoundaryInfoConstPtr
 */
inline const zmap::LaneBoundaryInfoConstPtr GetLaneBoundaryById(
    const std::shared_ptr<zmap::ZpilotMapInfo> new_hdmap,
    const zark::hdmap_new::Id &id, const bool need_lock = true) {
  zmap::MapFeaturePtr map_info;
  zmap::LaneBoundaryInfoConstPtr bound_info = nullptr;
  if (new_hdmap == nullptr) return bound_info;
  std::unique_lock<std::mutex> lock(HDMapUtil::map_service_mutex_,
                                    std::defer_lock);
  if (need_lock) lock.lock();
  const zmap::MapFeatureType lb_type = zmap::MapFeatureType::kLaneBoundaryInfo;
  if (new_hdmap->GetFeatureById(id, lb_type, &map_info)) {
    bound_info =
        std::dynamic_pointer_cast<const zmap::LaneBoundaryInfo>(map_info);
  }
  return bound_info;
}

/**
 * @brief get all junctions in certain range
 * @param new_hdmap hdmap_service ptr
 * @param point the central point of the range
 * @param distance the search radius
 * @param junctions store all junctions in target range
 * @return 0:success, otherwise failed
 */
inline int GetJunctions(const std::shared_ptr<zmap::ZpilotMapInfo> new_hdmap,
                        const hdmap_new::Point2D &point, double distance,
                        std::vector<zmap::JunctionInfoConstPtr> *junctions,
                        const bool need_lock = true) {
  if (!new_hdmap || !junctions) return -1;
  std::unique_lock<std::mutex> lock(HDMapUtil::map_service_mutex_,
                                    std::defer_lock);
  if (need_lock) lock.lock();
  std::vector<zmap::MapFeaturePtr> map_info;
  const zmap::MapFeatureType lb_type = zmap::MapFeatureType::kJunctionInfo;
  if (new_hdmap->GetFeaturesByRangeWithin(point, distance, lb_type,
                                          &map_info)) {
    for (const auto &feature : map_info) {
      zmap::JunctionInfoConstPtr junction_info =
          std::dynamic_pointer_cast<const zmap::JunctionInfo>(feature);
      junctions->emplace_back(junction_info);
    }
  }
  if (junctions->size() > 0) {
    return 0;
  } else {
    return -1;
  }
}

}  // namespace hdmap
}  // namespace planning
}  // namespace zark
