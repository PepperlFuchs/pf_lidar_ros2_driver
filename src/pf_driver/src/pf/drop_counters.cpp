#include "pf_driver/pf/drop_counters.h"

#include <optional>
#include <utility>
#include <vector>

DropCounters::DropCounters(Reporter reporter) : reporter_(std::move(reporter))
{
}

void DropCounters::recordDrop(DropReason reason, const DropDetails& details)
{
  std::optional<DropReport> report;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = reasons_.at(static_cast<std::size_t>(reason));
    ++state.total;
    state.active = true;
    if (state.armed)
    {
      state.armed = false;
      report = DropReport{ reason, 1, state.total, true, details };
    }
    else
    {
      ++state.pending;
    }
  }
  if (report && reporter_)
  {
    reporter_(*report);
  }
}

void DropCounters::recordPublication()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++published_;
}

void DropCounters::setQueueCapacity(std::size_t capacity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_capacity_ = capacity;
}

DropSnapshot DropCounters::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  DropSnapshot result;
  result.scans_published = published_;
  result.packet_queue_capacity = queue_capacity_;
  for (std::size_t i = 0; i < reasons_.size(); ++i)
  {
    result.drops[i] = reasons_[i].total;
  }
  return result;
}

void DropCounters::tick()
{
  std::vector<DropReport> reports;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < reasons_.size(); ++i)
    {
      auto& state = reasons_[i];
      if (state.pending != 0)
      {
        reports.push_back({ static_cast<DropReason>(i), state.pending, state.total, false, {} });
        state.pending = 0;
      }
      if (!state.active)
      {
        state.armed = true;
      }
      state.active = false;
    }
  }
  if (reporter_)
  {
    for (const auto& report : reports)
    {
      reporter_(report);
    }
  }
}

bool RecentScanAttribution::contains(uint16_t number) const
{
  return numbers_.find(number) != numbers_.end();
}

bool RecentScanAttribution::remember(uint16_t number)
{
  if (!numbers_.insert(number).second)
  {
    return false;
  }
  order_.push_back(number);
  if (order_.size() > CAPACITY)
  {
    numbers_.erase(order_.front());
    order_.pop_front();
  }
  return true;
}
