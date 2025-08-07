#include "speed/slt_info.h"

#include <algorithm>
#include <cmath>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "math/geometry/segment2d.h"
#include "math/util.h"

namespace e2e_noa::planning {

void SltInfo::FormatSltInfo(const std::vector<NearestSlPoint>& sl_points) {
  near_sl_points_.clear();
  if (sl_points.size() < 2) {
    return;
  }
  for (double t = 0.0; t < t_length_ + 1e-3; t += t_interval_) {
    if (t < sl_points.front().t) {
      near_sl_points_.push_back(std::nullopt);
    } else if (t > sl_points.back().t) {
      near_sl_points_.push_back(std::nullopt);
    } else {
      const auto it = std::lower_bound(
          sl_points.begin(), sl_points.end(), t,
          [](const NearestSlPoint& p, double t) { return p.t < t; });
      if (it == sl_points.begin() || it == sl_points.end()) {
        near_sl_points_.push_back(std::nullopt);
        continue;
      }
      NearestSlPoint near_pt;
      const auto it_pre = it - 1;
      near_pt.av_heading =
          Lerp(it_pre->av_heading, it_pre->t, it->av_heading, it->t, t, true);
      near_pt.av_s = Lerp(it_pre->av_s, it_pre->t, it->av_s, it->t, t, true);
      near_pt.lat_dist =
          Lerp(it_pre->lat_dist, it_pre->t, it->lat_dist, it->t, t, true);
      near_pt.obj_heading =
          Lerp(it_pre->obj_heading, it_pre->t, it->obj_heading, it->t, t, true);
      near_pt.obj_v = Lerp(it_pre->obj_v, it_pre->t, it->obj_v, it->t, t, true);
      near_pt.obj_vl =
          Lerp(it_pre->obj_vl, it_pre->t, it->obj_vl, it->t, t, true);
      near_pt.obj_idx = CeilToInteger(
          Lerp(it_pre->obj_idx, it_pre->t, it->obj_idx, it->t, t, true));
      near_pt.t = t;
      near_sl_points_.push_back(near_pt);
      valid_pt_num_++;
    }
  }
  return;
}

std::optional<std::string> SltInfo::RecoverObjectId(
    const std::string& traj_id) {
  if (const auto found = traj_id.find("-idx"); found == std::string::npos) {
    LOG(ERROR) << "traj_id of st_object [" << traj_id
               << "] should contain \"-idx\" but not!";
    return std::nullopt;
  } else {
    return traj_id.substr(0, found);
  }
}
}  // namespace e2e_noa::planning
