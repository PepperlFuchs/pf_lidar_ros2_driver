#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <unordered_set>

enum class DropReason : std::size_t
{
  MISSING_FIRST_PACKET,
  INCOMPLETE_SCAN,
  QUEUE_FULL,
  COUNT
};

struct DropDetails
{
  uint16_t scan_number = 0;
  uint16_t packet_number = 0;
  uint16_t next_scan_number = 0;
  uint64_t received_points = 0;
  uint64_t expected_points = 0;
  uint64_t queue_capacity = 0;
};

struct DropReport
{
  DropReason reason;
  uint64_t count;
  uint64_t total;
  bool first;
  DropDetails details;
};

struct DropSnapshot
{
  uint64_t scans_published = 0;
  std::array<uint64_t, static_cast<std::size_t>(DropReason::COUNT)> drops{};
  // Zero until the first Pipeline supplies its actual capacity.
  uint64_t packet_queue_capacity = 0;
};

// PFInterface shares one instance with successive publishers and pipelines.
// Report callbacks run after the accounting mutex is released and may overlap.
class DropCounters
{
public:
  using Reporter = std::function<void(const DropReport&)>;

  explicit DropCounters(Reporter reporter = {});
  void recordDrop(DropReason reason, const DropDetails& details = {});
  void recordPublication();
  void setQueueCapacity(std::size_t capacity);
  DropSnapshot snapshot() const;
  // The caller invokes this once per second, independently of diagnostics.
  void tick();

private:
  struct ReasonState
  {
    uint64_t total = 0;
    uint64_t pending = 0;
    bool active = false;
    bool armed = true;
  };

  mutable std::mutex mutex_;
  std::array<ReasonState, static_cast<std::size_t>(DropReason::COUNT)> reasons_{};
  uint64_t published_ = 0;
  uint64_t queue_capacity_ = 0;
  const Reporter reporter_;
};

// A scan number is not globally unique. Repeated observations do not refresh
// this bounded history. The assembler retains its own attribution separately.
class RecentScanAttribution
{
public:
  static constexpr std::size_t CAPACITY = 256;

  bool contains(uint16_t number) const;
  // Returns true only when this number was absent from the retained history.
  bool remember(uint16_t number);

private:
  std::deque<uint16_t> order_;
  std::unordered_set<uint16_t> numbers_;
};
