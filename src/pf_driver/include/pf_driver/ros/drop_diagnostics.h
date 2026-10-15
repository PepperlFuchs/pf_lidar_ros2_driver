#pragma once

#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pf_driver/pf/drop_counters.h"

diagnostic_msgs::msg::DiagnosticStatus dropDiagnosticStatus(const DropSnapshot& current, const DropSnapshot& previous,
                                                            const std::string& name, const std::string& hardware_id);

// Owns separate wall timers for diagnostics and one-second log summaries.
// Timer callbacks retain their data independently of this object's lifetime.
class DropDiagnostics
{
public:
  explicit DropDiagnostics(const std::shared_ptr<rclcpp::Node>& node);
  ~DropDiagnostics();

  std::shared_ptr<DropCounters> counters() const;
  void setHardwareId(const std::string& hardware_id);

private:
  struct State;
  std::shared_ptr<State> state_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};
