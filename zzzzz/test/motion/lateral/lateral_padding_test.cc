#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/motion/lateral/lateral_padding.h"
#include "apps/planning/src/common/lookup_table.h"
#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

using namespace zark::planning;
using namespace std;

TEST(LateralPaddingTest, GetSoftPadding_test) {
  LateralOptimizerConfig lateral_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/lateral_optimizer_config_test.json";
  Config config{config_file};
  std::string config_type = "table_config";
  std::ifstream jdata(config_file);
  lateral_config =
      config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
  LateralLookupTables lateral_tables(lateral_config);
  LateralPadding lateral_padding;

  EXPECT_DOUBLE_EQ(lateral_padding.GetSoftPadding(lateral_tables, 8.0), 0.1);
  EXPECT_DOUBLE_EQ(lateral_padding.GetSoftPadding(lateral_tables, 35.0), 0.1);
  EXPECT_DOUBLE_EQ(lateral_padding.GetSoftPadding(lateral_tables, -1.0), 0.1);
}

TEST(LateralPaddingTest, GetStiffPadding_test) {
  LateralOptimizerConfig lateral_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/lateral_optimizer_config_test.json";
  Config config{config_file};
  std::string config_type = "table_config";
  std::ifstream jdata(config_file);
  lateral_config =
      config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
  LateralLookupTables lateral_tables(lateral_config);
  LateralPadding lateral_padding;

  EXPECT_DOUBLE_EQ(
      lateral_padding.GetStiffPadding(lateral_tables, perception::ST_CAR), 0.1);
  EXPECT_DOUBLE_EQ(
      lateral_padding.GetStiffPadding(lateral_tables, perception::ST_CAR, true),
      0.1);
  EXPECT_DOUBLE_EQ(
      lateral_padding.GetStiffPadding(lateral_tables, perception::ST_BIG_BUS),
      0.1);
  EXPECT_DOUBLE_EQ(lateral_padding.GetStiffPadding(lateral_tables,
                                                   perception::ST_PEDESTRIAN),
                   0.1);
  EXPECT_DOUBLE_EQ(
      lateral_padding.GetStiffPadding(lateral_tables, perception::ST_CYCLIST),
      0.1);
  EXPECT_DOUBLE_EQ(
      lateral_padding.GetStiffPadding(lateral_tables, perception::ST_UNKNOWN),
      0.1);
}
