/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <string>

#include "apps/planning/src/basic_msgs/error_code.h"

namespace zark {
namespace planning {
namespace common {
class Header {
 public:
  // Default constructor
  Header()
      : timestamp_sec_(0.0),
        module_name_(""),
        sequence_num_(0),
        lidar_timestamp_(0),
        camera_timestamp_(0),
        radar_timestamp_(0),
        version_(0),
        status_(),
        frame_id_("") {}

  // Parameterized constructor
  Header(double timestamp, const std::string& module, uint32_t sequence,
         uint64_t lidar_timestamp, uint64_t camera_timestamp,
         uint64_t radar_timestamp, uint32_t version, StatusPb status,
         const std::string& frame)
      : timestamp_sec_(timestamp),
        module_name_(module),
        sequence_num_(sequence),
        lidar_timestamp_(lidar_timestamp),
        camera_timestamp_(camera_timestamp),
        radar_timestamp_(radar_timestamp),
        version_(version),
        status_(status),
        frame_id_(frame) {}

  // Getter functions
  double timestamp_sec() const { return timestamp_sec_; }
  const std::string& module_name() const { return module_name_; }
  uint32_t sequence_num() const { return sequence_num_; }
  uint64_t lidar_timestamp() const { return lidar_timestamp_; }
  uint64_t camera_timestamp() const { return camera_timestamp_; }
  uint64_t radar_timestamp() const { return radar_timestamp_; }
  uint32_t version() const { return version_; }
  StatusPb status() const { return status_; }
  const std::string& frame_id() const { return frame_id_; }

  // Setter functions
  void set_timestamp_sec(double timestamp) { timestamp_sec_ = timestamp; }
  void set_module_name(const std::string& module) { module_name_ = module; }
  void set_sequence_num(uint32_t sequence) { sequence_num_ = sequence; }
  void set_lidar_timestamp(uint64_t lidar_timestamp) {
    lidar_timestamp_ = lidar_timestamp;
  }
  void set_camera_timestamp(uint64_t camera_timestamp) {
    camera_timestamp_ = camera_timestamp;
  }
  void set_radar_timestamp(uint64_t radar_timestamp) {
    radar_timestamp_ = radar_timestamp;
  }
  void set_version(uint32_t version) { version_ = version; }
  void set_status(StatusPb status) { status_ = status; }
  void set_frame_id(const std::string& frame) { frame_id_ = frame; }
  void CopyFrom(const Header& other) { *this = other; }
  std::string DebugString() { return "Header"; }

  StatusPb* mutable_status() { return &status_; }

  void Clear() {
    timestamp_sec_ = 0.0;
    module_name_ = "";
    sequence_num_ = 0;
    lidar_timestamp_ = 0;
    camera_timestamp_ = 0;
    radar_timestamp_ = 0;
    version_ = 0;
    status_.Clear();
    frame_id_ = "0";
  }

 private:
  double timestamp_sec_;     // Message publishing time in seconds.
  std::string module_name_;  // Module name
  uint32_t sequence_num_;
  uint64_t lidar_timestamp_;
  uint64_t camera_timestamp_;
  uint64_t radar_timestamp_;
  uint32_t version_;
  StatusPb status_;
  std::string frame_id_;
};

}  // namespace common

}  // namespace planning
}  // namespace zark
