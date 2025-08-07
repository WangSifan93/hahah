/******************************************************************************
 * Copyright 2018 The zark Authors. All Rights Reserved.
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
 * @file
 **/

#include "apps/planning/src/common/trajectory/publishable_trajectory.h"

#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/common/util/util.h"
#include "gtest/gtest.h"

using namespace zark::planning;
using namespace std;
TEST(PublishableTrajectoryTest, basic_test) {
  ADCTrajectory trajectory;
  std::vector<::common::TrajectoryPoint> trajectory_points;
  std::vector<::common::PathPoint> path_points;
  ::common::TrajectoryPoint trajectory_point;
  ::common::PathPoint path_point;
  trajectory.set_is_replan(true);
  // trajectory.set_gear(canbus::Chassis::GearPosition::GEAR_DRIVE);
  string filename = "testdata/standard_trajectory.pb.txt";
  fstream fin;
  fin.open(filename.c_str(), ios::in);
  if (!fin.is_open()) {
    cout << "不能打开文件" << filename << endl;
  }
  string tmp, var;
  size_t n, length;
  while (getline(fin, tmp)) {
    var = "total_path_length: ";
    length = var.length();
    n = tmp.rfind(var);
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      trajectory.set_total_path_length(atof(tmp.c_str()));
      continue;
    }
    var = "total_path_time: ";
    length = var.length();
    n = tmp.rfind(var);
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      trajectory.set_total_path_time(atof(tmp.c_str()));
      continue;
    }
    var = " x: ";
    length = var.length();
    n = tmp.rfind(" x: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      path_point.set_x(atof(tmp.c_str()));
      continue;
    }
    var = " y: ";
    length = var.length();
    n = tmp.rfind(" y: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      path_point.set_y(atof(tmp.c_str()));
      continue;
    }
    var = " theta: ";
    length = var.length();
    n = tmp.rfind("theta: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      path_point.set_theta(atof(tmp.c_str()));
      continue;
    }
    var = " kappa: ";
    length = var.length();
    n = tmp.rfind(" kappa: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      path_point.set_kappa(atof(tmp.c_str()));
      continue;
    }
    var = " s: ";
    length = var.length();
    n = tmp.rfind(" s: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      path_point.set_s(atof(tmp.c_str()));
      continue;
    }
    var = " dkappa: ";
    length = var.length();
    n = tmp.rfind(" dkappa: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      path_point.set_dkappa(atof(tmp.c_str()));
      continue;
    }
    var = " ddkappa: ";
    length = var.length();
    n = tmp.rfind(" ddkappa: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      path_point.set_ddkappa(atof(tmp.c_str()));
      continue;
    }
    var = " v: ";
    length = var.length();
    n = tmp.rfind(" v: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      trajectory_point.set_v(atof(tmp.c_str()));
      continue;
    }
    var = " a: ";
    length = var.length();
    n = tmp.rfind(" a: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      trajectory_point.set_a(atof(tmp.c_str()));
      continue;
    }
    var = " relative_time: ";
    length = var.length();
    n = tmp.rfind(" relative_time: ");
    if (n != string::npos) {
      tmp = tmp.substr(n + length);
      trajectory_point.set_relative_time(atof(tmp.c_str()));
      trajectory_point.set_path_point(path_point);
      trajectory_points.emplace_back(trajectory_point);
      path_points.emplace_back(path_point);
      continue;
    }
    n = tmp.rfind("trajectory_point");
    if (n != string::npos) {
      trajectory_point = ::common::TrajectoryPoint();
      path_point = ::common::PathPoint();
      continue;
    }
  }
  trajectory.set_trajectory_point(trajectory_points);
  trajectory.set_path_point(path_points);
  ;
  DiscretizedTrajectory discretized_trajectory(trajectory);

  PublishableTrajectory publishable_trajectory(12349834.26,
                                               discretized_trajectory);
  EXPECT_EQ(publishable_trajectory.header_time(), 12349834.26);

  ADCTrajectory output_trajectory;
  publishable_trajectory.PopulateTrajectoryProtobuf(&output_trajectory);

  for (size_t i = 0; i < output_trajectory.trajectory_point().size(); ++i) {
    double x = output_trajectory.trajectory_point().at(i).relative_time();
    EXPECT_DOUBLE_EQ(
        output_trajectory.trajectory_point().at(i).path_point().x(),
        trajectory.trajectory_point().at(i).path_point().x());
    EXPECT_DOUBLE_EQ(
        output_trajectory.trajectory_point().at(i).path_point().y(),
        trajectory.trajectory_point().at(i).path_point().y());
    EXPECT_DOUBLE_EQ(
        output_trajectory.trajectory_point().at(i).path_point().theta(),
        trajectory.trajectory_point().at(i).path_point().theta());
    EXPECT_DOUBLE_EQ(
        output_trajectory.trajectory_point().at(i).path_point().kappa(),
        trajectory.trajectory_point().at(i).path_point().kappa());
    EXPECT_DOUBLE_EQ(
        output_trajectory.trajectory_point().at(i).path_point().dkappa(),
        trajectory.trajectory_point().at(i).path_point().dkappa());
    EXPECT_DOUBLE_EQ(
        output_trajectory.trajectory_point().at(i).path_point().ddkappa(),
        trajectory.trajectory_point().at(i).path_point().ddkappa());
    EXPECT_DOUBLE_EQ(output_trajectory.trajectory_point().at(i).relative_time(),
                     trajectory.trajectory_point().at(i).relative_time());
    EXPECT_DOUBLE_EQ(output_trajectory.trajectory_point().at(i).v(),
                     trajectory.trajectory_point().at(i).v());
    EXPECT_DOUBLE_EQ(output_trajectory.trajectory_point().at(i).a(),
                     trajectory.trajectory_point().at(i).a());
  }
}
