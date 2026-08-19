#include <gtest/gtest.h>

#include <string>

#include "edgepick_perception/perception_metrics.hpp"

namespace edgepick_perception
{
namespace
{

TEST(RunningStatisticsTest, TracksMeanMinMaxAndSampleStddev)
{
  RunningStatistics statistics;

  statistics.add(1.0);
  statistics.add(2.0);
  statistics.add(3.0);

  EXPECT_EQ(statistics.count, 3U);
  EXPECT_DOUBLE_EQ(statistics.min, 1.0);
  EXPECT_DOUBLE_EQ(statistics.max, 3.0);
  EXPECT_DOUBLE_EQ(statistics.mean, 2.0);
  EXPECT_DOUBLE_EQ(statistics.variance(), 1.0);
  EXPECT_DOUBLE_EQ(statistics.stddev(), 1.0);
}

TEST(PerceptionMetricsAccumulatorTest, TracksDetectionMessagesAndLatency)
{
  PerceptionMetricsAccumulator metrics;

  metrics.record_detection_message(10.0, 10.125, 3U, 0.92);
  metrics.record_detection_message(std::nullopt, 11.0, 0U, std::nullopt);

  const auto snapshot = metrics.snapshot();
  EXPECT_EQ(snapshot.detection_messages, 2U);
  EXPECT_EQ(snapshot.detection_candidates, 3U);
  EXPECT_EQ(snapshot.empty_detection_messages, 1U);
  ASSERT_EQ(snapshot.detection_latency_ms.count, 1U);
  EXPECT_NEAR(snapshot.detection_latency_ms.mean, 125.0, 1.0e-9);
  ASSERT_EQ(snapshot.detection_best_score.count, 1U);
  EXPECT_DOUBLE_EQ(snapshot.detection_best_score.mean, 0.92);
}

TEST(PerceptionMetricsAccumulatorTest, TracksTargetStabilityAndEvents)
{
  PerceptionMetricsAccumulator metrics;

  metrics.record_target_point(20.0, 20.010, 0.0, 0.0, 0.50);
  metrics.record_target_point(20.1, 20.120, 0.03, 0.04, 0.50);
  metrics.record_task_event("target_acquired");
  metrics.record_task_event("target_lost");
  metrics.record_task_event("planning_started");

  const auto snapshot = metrics.snapshot();
  EXPECT_EQ(snapshot.target_points, 2U);
  ASSERT_EQ(snapshot.target_latency_ms.count, 2U);
  EXPECT_NEAR(snapshot.target_latency_ms.mean, 15.0, 1.0e-9);
  ASSERT_EQ(snapshot.target_step_m.count, 1U);
  EXPECT_NEAR(snapshot.target_step_m.mean, 0.05, 1.0e-9);
  EXPECT_EQ(snapshot.target_acquired_events, 1U);
  EXPECT_EQ(snapshot.target_lost_events, 1U);
  EXPECT_EQ(snapshot.other_task_events, 1U);
}

TEST(PerceptionMetricsAccumulatorTest, FormatsStableOneLineSummary)
{
  PerceptionMetricsAccumulator metrics;
  metrics.record_detection_message(1.0, 1.050, 1U, 0.80);
  metrics.record_target_point(1.0, 1.060, 0.1, 0.2, 0.3);
  metrics.record_task_event("target_acquired");

  const std::string summary = format_metrics_summary(metrics.snapshot());

  EXPECT_NE(summary.find("detections=1"), std::string::npos);
  EXPECT_NE(summary.find("detection_latency_ms_mean=50.0"), std::string::npos);
  EXPECT_NE(summary.find("target_points=1"), std::string::npos);
  EXPECT_NE(summary.find("target_acquired_events=1"), std::string::npos);
}

}  // namespace
}  // namespace edgepick_perception
