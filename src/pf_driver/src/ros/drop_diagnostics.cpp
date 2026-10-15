#include "pf_driver/ros/drop_diagnostics.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace
{

constexpr const char* REASON_NAMES[] = { "scans_dropped_missing_first_packet", "scans_dropped_incomplete",
                                         "packets_dropped_queue_full" };

void logDrop(const rclcpp::Logger& logger, const DropReport& report)
{
  const auto reason = static_cast<std::size_t>(report.reason);
  if (!report.first)
  {
    RCLCPP_WARN(logger, "%s: %lu additional drops since the previous summary; total %lu", REASON_NAMES[reason],
                static_cast<unsigned long>(report.count), static_cast<unsigned long>(report.total));
    return;
  }
  const auto& d = report.details;
  switch (report.reason)
  {
    case DropReason::MISSING_FIRST_PACKET:
      RCLCPP_WARN(logger, "Discarded packet %u of scan %u: its first packet has not started a scan (total %lu)",
                  d.packet_number, d.scan_number, static_cast<unsigned long>(report.total));
      break;
    case DropReason::INCOMPLETE_SCAN:
      RCLCPP_WARN(logger, "Discarded incomplete scan %u (%lu of %lu received points) when scan %u started (total %lu)",
                  d.scan_number, static_cast<unsigned long>(d.received_points),
                  static_cast<unsigned long>(d.expected_points), d.next_scan_number,
                  static_cast<unsigned long>(report.total));
      break;
    case DropReason::QUEUE_FULL:
      RCLCPP_WARN(logger, "Discarded packet %u of scan %u: packet queue is full at %lu packets (total %lu)",
                  d.packet_number, d.scan_number, static_cast<unsigned long>(d.queue_capacity),
                  static_cast<unsigned long>(report.total));
      break;
    case DropReason::COUNT:
      break;
  }
}

}  // namespace

diagnostic_msgs::msg::DiagnosticStatus dropDiagnosticStatus(const DropSnapshot& current, const DropSnapshot& previous,
                                                            const std::string& name, const std::string& hardware_id)
{
  diagnostic_msgs::msg::DiagnosticStatus result;
  result.name = name;
  result.hardware_id = hardware_id;
  const uint64_t scans = current.drops[0] - previous.drops[0] + current.drops[1] - previous.drops[1];
  const uint64_t packets = current.drops[2] - previous.drops[2];
  result.level = (scans != 0 || packets != 0) ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
                                                diagnostic_msgs::msg::DiagnosticStatus::OK;
  result.message = std::to_string(scans) + " scans and " + std::to_string(packets) +
                   " packets affected by drops since the previous update";
  const auto addValue = [&result](const std::string& key, uint64_t value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = std::to_string(value);
    result.values.push_back(std::move(entry));
  };
  addValue("scans_published", current.scans_published);
  for (std::size_t i = 0; i < current.drops.size(); ++i)
  {
    addValue(REASON_NAMES[i], current.drops[i]);
  }
  addValue("packet_queue_capacity", current.packet_queue_capacity);
  return result;
}

struct DropDiagnostics::State
{
  std::shared_ptr<DropCounters> counters;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher;
  std::string name;
  std::mutex hardware_mutex;
  std::string hardware_id;
  // Only the mutually exclusive diagnostics timer accesses this value.
  DropSnapshot previous;
};

DropDiagnostics::DropDiagnostics(const std::shared_ptr<rclcpp::Node>& node) : state_(std::make_shared<State>())
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Wall-clock seconds between drop diagnostics. Log summaries remain at one second.";
  descriptor.read_only = true;
  const double period = node->declare_parameter<double>("diagnostics.period", 1.0, descriptor);
  const long double nanoseconds = static_cast<long double>(period) * 1000000000.0L;
  if (!std::isfinite(period) || nanoseconds < 1.0L ||
      nanoseconds > static_cast<long double>(std::numeric_limits<int64_t>::max()))
  {
    throw std::invalid_argument("diagnostics.period must be finite, positive and representable in nanoseconds");
  }
  const auto duration = std::chrono::nanoseconds(static_cast<int64_t>(nanoseconds));
  const auto logger = node->get_logger();
  state_->counters = std::make_shared<DropCounters>([logger](const DropReport& report) { logDrop(logger, report); });
  state_->name = std::string(node->get_name()) + ": dropped scans and packets";
  state_->hardware_id = "unknown";
  node->get_parameter("scanner_ip", state_->hardware_id);
  state_->publisher = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto state = state_;
  const auto clock = node->get_clock();
  diagnostics_timer_ = node->create_wall_timer(
      duration,
      [state, clock] {
        const auto current = state->counters->snapshot();
        std::string hardware_id;
        {
          std::lock_guard<std::mutex> lock(state->hardware_mutex);
          hardware_id = state->hardware_id;
        }
        diagnostic_msgs::msg::DiagnosticArray message;
        message.header.stamp = clock->now();
        message.status.push_back(dropDiagnosticStatus(current, state->previous, state->name, hardware_id));
        state->publisher->publish(message);
        state->previous = current;
      },
      group_);
  summary_timer_ = node->create_wall_timer(std::chrono::seconds(1), [state] { state->counters->tick(); }, group_);
}

DropDiagnostics::~DropDiagnostics()
{
  if (diagnostics_timer_)
  {
    diagnostics_timer_->cancel();
  }
  if (summary_timer_)
  {
    summary_timer_->cancel();
  }
}

std::shared_ptr<DropCounters> DropDiagnostics::counters() const
{
  return state_->counters;
}

void DropDiagnostics::setHardwareId(const std::string& hardware_id)
{
  std::lock_guard<std::mutex> lock(state_->hardware_mutex);
  state_->hardware_id = hardware_id;
}
