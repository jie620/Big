#include <gtest/gtest.h>

#include <vector>

#include "edgepick_perception/target_detection_selection.hpp"

namespace edgepick_perception
{
namespace
{

TEST(TargetDetectionSelectionTest, SelectsHighestScoringCandidate)
{
  const std::vector<TargetDetectionCandidate> candidates{
    TargetDetectionCandidate{1U, "target", 0.70, 100.0, 120.0, 30.0, 40.0},
    TargetDetectionCandidate{1U, "target", 0.95, 220.0, 240.0, 50.0, 60.0},
    TargetDetectionCandidate{1U, "target", 0.80, 300.0, 320.0, 70.0, 80.0}};

  const auto selected = select_target_detection(candidates, TargetDetectionSelectionConfig{});

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->score, 0.95);
  EXPECT_EQ(detection_center_pixel(*selected).u, 220);
  EXPECT_EQ(detection_center_pixel(*selected).v, 240);
}

TEST(TargetDetectionSelectionTest, AppliesClassAndLabelFilters)
{
  TargetDetectionSelectionConfig config;
  config.target_class_id = 7U;
  config.target_label = "apple";

  const std::vector<TargetDetectionCandidate> candidates{
    TargetDetectionCandidate{7U, "box", 0.99, 100.0, 100.0, 20.0, 20.0},
    TargetDetectionCandidate{3U, "apple", 0.99, 120.0, 100.0, 20.0, 20.0},
    TargetDetectionCandidate{7U, "apple", 0.80, 140.0, 100.0, 20.0, 20.0}};

  const auto selected = select_target_detection(candidates, config);

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->class_id, 7U);
  EXPECT_EQ(selected->label, "apple");
  EXPECT_EQ(selected->center_u, 140.0);
}

TEST(TargetDetectionSelectionTest, RejectsLowScoreAndInvalidGeometry)
{
  TargetDetectionSelectionConfig config;
  config.min_score = 0.60;

  const std::vector<TargetDetectionCandidate> candidates{
    TargetDetectionCandidate{1U, "target", 0.59, 100.0, 100.0, 20.0, 20.0},
    TargetDetectionCandidate{1U, "target", 0.90, -1.0, 100.0, 20.0, 20.0},
    TargetDetectionCandidate{1U, "target", 0.90, 100.0, 100.0, 0.0, 20.0}};

  EXPECT_FALSE(select_target_detection(candidates, config).has_value());
}

TEST(TargetDetectionSelectionTest, UsesAreaAsDeterministicTieBreaker)
{
  const std::vector<TargetDetectionCandidate> candidates{
    TargetDetectionCandidate{1U, "target", 0.80, 100.0, 100.0, 20.0, 20.0},
    TargetDetectionCandidate{1U, "target", 0.80, 120.0, 100.0, 40.0, 40.0}};

  const auto selected = select_target_detection(candidates, TargetDetectionSelectionConfig{});

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->center_u, 120.0);
}

TEST(TargetDetectionSelectionTest, RoundsDetectionCenterToPixel)
{
  const Pixel pixel =
    detection_center_pixel(TargetDetectionCandidate{1U, "target", 0.90, 319.6, 239.4, 20.0, 20.0});

  EXPECT_EQ(pixel.u, 320);
  EXPECT_EQ(pixel.v, 239);
}

}  // namespace
}  // namespace edgepick_perception
