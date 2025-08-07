/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <map>
#include <string>
#include <vector>

#include "header.h"
#include "pnc_point.h"
#include "messages/mapfusion/all_map.pb.h"
#include "messages/localization/localization.pb.h"
#include "messages/perception/obstacle.pb.h"

namespace zark {
namespace planning {

namespace relative_map {

// Struct
class NavigationPath {
 public:
  // Default constructor
  NavigationPath() : path_priority_(0) {}

  // Getter and Setter functions
  const ::common::Path& path() const { return path_; }
  uint32_t path_priority() const { return path_priority_; }

  void set_path(const ::common::Path& path) { path_ = path; }
  void set_path_priority(uint32_t priority) { path_priority_ = priority; }

 private:
  ::common::Path path_;
  uint32_t path_priority_;
};

class NavigationInfo {
 public:
  // Getter and Setter functions
  const ::common::Header& header() const { return header_; }
  const std::vector<NavigationPath>& navigation_path() const {
    return navigation_path_;
  }

  void set_header(const ::common::Header& header) { header_ = header; }
  void set_navigation_path(const std::vector<NavigationPath>& paths) {
    navigation_path_ = paths;
  }

 private:
  ::common::Header header_;
  std::vector<NavigationPath> navigation_path_;
};

// The map message in transmission format.
class MapMsg {
 public:
  // Getter and Setter functions
  const ::common::Header& header() const { return header_; }
  const zark::hdmap::Map& hdmap() const { return hdmap_; }
  const std::map<std::string, NavigationPath>& navigation_path() const {
    return navigation_path_;
  }
  const zark::perception::LaneMarkers& lane_marker() const {
    return lane_marker_;
  }
  const zark::localization::LocalizationInfo& localization() const {
    return localization_;
  }

  void set_header(const ::common::Header& header) { header_ = header; }
  void set_hdmap(const zark::hdmap::Map& hdmap) { hdmap_ = hdmap; }
  void set_navigation_path(const std::map<std::string, NavigationPath>& paths) {
    navigation_path_ = paths;
  }
  void set_lane_marker(const zark::perception::LaneMarkers& lane_marker) {
    lane_marker_ = lane_marker;
  }
  void set_localization(
      const zark::localization::LocalizationInfo& localization) {
    localization_ = localization;
  }

 private:
  ::common::Header header_;
  // Coordination: FLU
  // x: Forward
  // y: Left
  // z: Up
  zark::hdmap::Map hdmap_;
  // key: type string; the lane id in hdmap
  // value: Navigation path; the reference line of the lane
  std::map<std::string, NavigationPath> navigation_path_;
  // lane marker info from perception
  zark::perception::LaneMarkers lane_marker_;
  // localization
  zark::localization::LocalizationInfo localization_;
};

}  // namespace relative_map

}  // namespace planning
}  // namespace zark
