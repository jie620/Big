#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace edgepick_perception
{

struct RunningStatistics
{
  std::uint64_t count{0U};
  double min{0.0};
  double max{0.0};
  double mean{0.0};
  double m2{0.0};

  void add(double value);
  double variance() const;
  double stddev() const;
};

struct PerceptionMetricsSnapshot
{
  std::uint64_t detection_messages{0U};
  std::uint64_t detection_candidates{0U};
  std::uint64_t empty_detection_messages{0U};
  RunningStatistics detection_latency_ms;
  RunningStatistics detection_best_score;

  std::uint64_t target_points{0U};
  RunningStatistics target_latency_ms;
  RunningStatistics target_step_m;
  RunningStatistics target_z_m;

  std::uint64_t target_acquired_events{0U};
  std::uint64_t target_lost_events{0U};
  std::uint64_t other_task_events{0U};
};

class PerceptionMetricsAccumulator
{
public:
  void record_detection_message(
    std::optional<double> source_stamp_s,
    double receipt_time_s,
    std::size_t candidate_count,
    std::optional<double> best_score);
  void record_target_point(
    std::optional<double> source_stamp_s,
    double receipt_time_s,
    double x_m,
    double y_m,
    double z_m);
  void record_task_event(const std::string & event_name);

  PerceptionMetricsSnapshot snapshot() const;

private:
  struct PointSample
  {
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};
  };

  static void record_latency_ms(
    RunningStatistics & statistics,
    std::optional<double> source_stamp_s,
    double receipt_time_s);

  PerceptionMetricsSnapshot snapshot_;
  std::optional<PointSample> previous_target_;
};

std::string format_metrics_summary(const PerceptionMetricsSnapshot & snapshot);

}  // namespace edgepick_perception
