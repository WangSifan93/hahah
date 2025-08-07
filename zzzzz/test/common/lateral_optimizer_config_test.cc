#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/common/lookup_table.h"
#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

using namespace zark::planning;

TEST(LateralOptimizerConfigTest, load_file_test) {
  LateralOptimizerConfig lateral_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/lateral_optimizer_config_test.json";
  Config config{config_file};
  std::string config_type = "table_config";
  std::ifstream jdata(config_file);
  lateral_optimizer_config =
      config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
  int res = lateral_optimizer_config.model.num_states;
  EXPECT_DOUBLE_EQ(res, 4);
}
