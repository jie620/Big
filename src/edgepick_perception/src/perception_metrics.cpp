#include "edgepick_perception/perception_metrics.hpp"

#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

namespace edgepick_perception
{
namespace
{

double square(double value)
{
  return value * value;
}

std::string format_stat_mean(const RunningStatistics & statistics, int precision)
{
  if (statistics.count == 0U) {
    return "n/a";
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << statistics.mean;
  return output.str();
}

std::string format_stat_stddev(const RunningStatistics & statistics, int precision)
{
  if (statistics.count < 2U) {
    return "n/a";
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << statistics.stddev();
  return output.str();
}

}  // namespace

void RunningStatistics::add(double value)
{
  if (!std::isfinite(value)) {
    return;
  }

  if (count == 0U) {
    min = value;
    max = value;
    mean = value;
    m2 = 0.0;
    count = 1U;
    return;
  }

  if (value < min) {
    min = value;
  }
  if (value > max) {
    max = value;
  }

  ++count;
  const double delta = value - mean;
  mean += delta / static_cast<double>(count);
  const double delta2 = value - mean;
  m2 += delta * delta2;
}

double RunningStatistics::variance() const
{
  if (count < 2U) {
    return 0.0;
  }
  return m2 / static_cast<double>(count - 1U);
}

double RunningStatistics::stddev() const
{
  return std::sqrt(variance());
}

void PerceptionMetricsAccumulator::record_detection_message(
  std::optional<double> source_stamp_s,
  double receipt_time_s,
  std::size_t candidate_count,
  std::optional<double> best_score)
{
  ++snapshot_.detection_messages;
  snapshot_.detection_candidates += static_cast<std::uint64_t>(candidate_count);
  if (candidate_count == 0U) {
    ++snapshot_.empty_detection_messages;
  }

  if (best_score.has_value()) {
    snapshot_.detection_best_score.add(*best_score);
  }
  record_latency_ms(snapshot_.detection_latency_ms, source_stamp_s, receipt_time_s);
}

void PerceptionMetricsAccumulator::record_target_point(
  std::optional<double> source_stamp_s,
  double receipt_time_s,
  double x_m,
  double y_m,
  double z_m)
{
  ++snapshot_.target_points;
  snapshot_.target_z_m.add(z_m);
  record_latency_ms(snapshot_.target_latency_ms, source_stamp_s, receipt_time_s);

  const PointSample current{x_m, y_m, z_m};
  if (previous_target_.has_value()) {
    const double step_m = std::sqrt(
      square(current.x_m - previous_target_->x_m) +
      square(current.y_m - previous_target_->y_m) +
      square(current.z_m - previous_target_->z_m));
    snapshot_.target_step_m.add(step_m);
  }
  previous_target_ = current;
}

void PerceptionMetricsAccumulator::record_task_event(const std::string & event_name)
{
  if (event_name == "target_acquired") {
    ++snapshot_.target_acquired_events;
    return;
  }
  if (event_name == "target_lost") {
    ++snapshot_.target_lost_events;
    return;
  }
  ++snapshot_.other_task_events;
}

PerceptionMetricsSnapshot PerceptionMetricsAccumulator::snapshot() const
{
  return snapshot_;
}

void PerceptionMetricsAccumulator::record_latency_ms(
  RunningStatistics & statistics,
  std::optional<double> source_stamp_s,
  double receipt_time_s)
{
  if (!source_stamp_s.has_value() || *source_stamp_s <= 0.0 || receipt_time_s < *source_stamp_s) {
    return;
  }
  statistics.add((receipt_time_s - *source_stamp_s) * 1000.0);
}

std::string format_metrics_summary(const PerceptionMetricsSnapshot & snapshot)
{
  std::ostringstream output;
  output << "detections=" << snapshot.detection_messages
         << " candidates=" << snapshot.detection_candidates
         << " empty_detections=" << snapshot.empty_detection_messages
         << " detection_latency_ms_mean="
         << format_stat_mean(snapshot.detection_latency_ms, 1)
         << " detection_score_mean=" << format_stat_mean(snapshot.detection_best_score, 3)
         << " target_points=" << snapshot.target_points
         << " target_latency_ms_mean=" << format_stat_mean(snapshot.target_latency_ms, 1)
         << " target_step_m_mean=" << format_stat_mean(snapshot.target_step_m, 4)
         << " target_step_m_stddev=" << format_stat_stddev(snapshot.target_step_m, 4)
         << " target_z_m_mean=" << format_stat_mean(snapshot.target_z_m, 4)
         << " target_z_m_stddev=" << format_stat_stddev(snapshot.target_z_m, 4)
         << " target_acquired_events=" << snapshot.target_acquired_events
         << " target_lost_events=" << snapshot.target_lost_events
         << " other_task_events=" << snapshot.other_task_events;
  return output.str();
}

}  // namespace edgepick_perception
