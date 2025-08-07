#include "apps/planning/src/motion/longitudinal/longitudinal_padding.h"

#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

using namespace zark::planning;
using namespace std;

TEST(LongitudinalPaddingTest, a_accel_table_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);

  EXPECT_DOUBLE_EQ(longitudinal_tables.a_accel_table().Evaluate(1.0), 1.45);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_accel_table().Evaluate(0.0), 1.4);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_accel_table().Evaluate(5.35), 1.67);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_accel_table().Evaluate(20), 0.5);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_accel_table().Evaluate(40), 0.5);
}

TEST(LongitudinalPaddingTest, a_max_stiff_table_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);

  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_stiff_table().Evaluate(1.0), 1.85);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_stiff_table().Evaluate(0.0), 1.8);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_stiff_table().Evaluate(5.35), 2.1);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_stiff_table().Evaluate(20), 0.6);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_stiff_table().Evaluate(40), 0.6);
}

TEST(LongitudinalPaddingTest, a_max_soft_table_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);

  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_soft_table().Evaluate(1.0), 1.45);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_soft_table().Evaluate(0.0), 1.4);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_soft_table().Evaluate(5.35), 1.67);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_soft_table().Evaluate(20), 0.5);
  EXPECT_DOUBLE_EQ(longitudinal_tables.a_max_soft_table().Evaluate(40), 0.5);
}

TEST(LongitudinalPaddingTest, front_soft_padding_table_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);

  EXPECT_DOUBLE_EQ(longitudinal_tables.front_soft_padding_table().Evaluate(1.0),
                   2.2);
  EXPECT_DOUBLE_EQ(longitudinal_tables.front_soft_padding_table().Evaluate(0.0),
                   1.0);
  EXPECT_DOUBLE_EQ(
      longitudinal_tables.front_soft_padding_table().Evaluate(10.0), 13.5);
  EXPECT_DOUBLE_EQ(
      longitudinal_tables.front_soft_padding_table().Evaluate(15.0), 20.0);
  EXPECT_DOUBLE_EQ(
      longitudinal_tables.front_soft_padding_table().Evaluate(40.0), 25.0);
}

TEST(LongitudinalPaddingTest, rear_soft_padding_table_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);

  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_soft_padding_table().Evaluate(1.0),
                   2.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_soft_padding_table().Evaluate(0.0),
                   1.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_soft_padding_table().Evaluate(10.0),
                   13.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_soft_padding_table().Evaluate(15.0),
                   20.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_soft_padding_table().Evaluate(40.0),
                   25.0);
}

TEST(LongitudinalPaddingTest, front_padding_multiplier_table_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);

  EXPECT_DOUBLE_EQ(
      longitudinal_tables.front_padding_relative_speed_multiplier_table()
          .Evaluate(1.0),
      0.1);
  EXPECT_DOUBLE_EQ(
      longitudinal_tables.front_padding_relative_speed_multiplier_table()
          .Evaluate(0.0),
      1.0);
  EXPECT_DOUBLE_EQ(
      longitudinal_tables.front_padding_relative_speed_multiplier_table()
          .Evaluate(0.333),
      0.5);
}

TEST(LongitudinalPaddingTest, rear_padding_multiplier_table_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);

  EXPECT_DOUBLE_EQ(
      longitudinal_tables.rear_padding_relative_speed_multiplier_table()
          .Evaluate(1.0),
      0.1);
  EXPECT_DOUBLE_EQ(
      longitudinal_tables.rear_padding_relative_speed_multiplier_table()
          .Evaluate(0.0),
      1.0);
  EXPECT_DOUBLE_EQ(
      longitudinal_tables.rear_padding_relative_speed_multiplier_table()
          .Evaluate(0.333),
      0.5);
}

TEST(LongitudinalPaddingTest, front_stiff_padding_table_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);

  EXPECT_DOUBLE_EQ(longitudinal_tables.front_stiff_padding_table().bicycle,
                   3.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.front_stiff_padding_table().regular_car,
                   4.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.front_stiff_padding_table().large_car,
                   6.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.front_stiff_padding_table().pedestrian,
                   3.0);
  EXPECT_DOUBLE_EQ(
      longitudinal_tables.front_stiff_padding_table().virtual_fence, -1.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.front_stiff_padding_table().others, 4.0);
}

TEST(LongitudinalPaddingTest, rear_stiff_padding_table_test) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);

  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_stiff_padding_table().bicycle, 3.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_stiff_padding_table().regular_car,
                   4.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_stiff_padding_table().large_car,
                   6.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_stiff_padding_table().pedestrian,
                   3.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_stiff_padding_table().virtual_fence,
                   -1.0);
  EXPECT_DOUBLE_EQ(longitudinal_tables.rear_stiff_padding_table().others, 4.0);
}

TEST(LongitudinalPaddingTest, TestGetFrontStiffPadding) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);
  LongitudinalPadding longitudinal_padding;
  EXPECT_DOUBLE_EQ(longitudinal_padding.GetFrontStiffPadding(
                       longitudinal_tables, perception::ST_CAR, false, 20, 20),
                   4.0);
  EXPECT_DOUBLE_EQ(longitudinal_padding.GetFrontStiffPadding(
                       longitudinal_tables, perception::ST_CAR, true, 20, 20),
                   -1.0);
  EXPECT_DOUBLE_EQ(
      longitudinal_padding.GetFrontStiffPadding(
          longitudinal_tables, perception::ST_HEAVY_TRUCK, false, 20, 20),
      6.0);
}

TEST(LongitudinalPaddingTest, TestGetRearStiffPadding) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);
  LongitudinalPadding longitudinal_padding;
  EXPECT_DOUBLE_EQ(longitudinal_padding.GetRearStiffPadding(
                       longitudinal_tables, perception::ST_CAR, false, 20, 20),
                   4.0);
  EXPECT_DOUBLE_EQ(longitudinal_padding.GetRearStiffPadding(
                       longitudinal_tables, perception::ST_CAR, true, 20, 20),
                   -1.0);
  EXPECT_DOUBLE_EQ(
      longitudinal_padding.GetRearStiffPadding(
          longitudinal_tables, perception::ST_HEAVY_TRUCK, false, 20, 20),
      6.0);
}

TEST(LongitudinalPaddingTest, TestGetFrontSoftPadding) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);
  LongitudinalPadding longitudinal_padding;
  EXPECT_DOUBLE_EQ(
      longitudinal_padding.GetFrontSoftPadding(longitudinal_tables, 10, 10),
      13.5);
  EXPECT_DOUBLE_EQ(
      longitudinal_padding.GetFrontSoftPadding(longitudinal_tables, 15, 20),
      20.0);
  EXPECT_NEAR(
      longitudinal_padding.GetFrontSoftPadding(longitudinal_tables, 21, 20),
      11.0, 0.1);
}

TEST(LongitudinalPaddingTest, TestGetRearSoftPadding) {
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  zark::planning::Config config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalLookupTables longitudinal_tables(longitudinal_optimizer_config);
  LongitudinalPadding longitudinal_padding;
  EXPECT_DOUBLE_EQ(
      longitudinal_padding.GetRearSoftPadding(longitudinal_tables, 10, 10),
      13.0);
  EXPECT_DOUBLE_EQ(
      longitudinal_padding.GetRearSoftPadding(longitudinal_tables, 15, 20),
      2.0);
  EXPECT_DOUBLE_EQ(
      longitudinal_padding.GetRearSoftPadding(longitudinal_tables, 21, 20),
      22.0);
}
