#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

using namespace zark::planning;

TEST(LongitudinalOptimizerConfigTest, load_file_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::string config_type = "table_config";
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  double res = longitudinal_optimizer_config.ref.a_accel_table.points_y.at(1);
  EXPECT_DOUBLE_EQ(res, 1.50);
}
