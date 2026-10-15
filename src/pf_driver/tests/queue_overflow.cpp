#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/pf/pipeline.h"
#include "pf_driver/tests/test_helper.h"

namespace
{

using namespace std::chrono_literals;

std::unique_ptr<PFPacket> parsedPacket(uint16_t scan, uint16_t number)
{
  std::ifstream file(get_dump_path() + "/dump_r2000_C.txt");
  std::string line;
  if (!std::getline(file, line))
  {
    throw std::runtime_error("cannot read the installed R2000 packet fixture");
  }
  auto bytes = hex_to_bytes(line);
  if (bytes.size() < 14)
  {
    throw std::runtime_error("R2000 packet fixture has a short header");
  }
  bytes[10] = static_cast<uint8_t>(scan);
  bytes[11] = static_cast<uint8_t>(scan >> 8);
  bytes[12] = static_cast<uint8_t>(number);
  bytes[13] = static_cast<uint8_t>(number >> 8);
  PFR2000_C_Parser parser;
  std::vector<std::unique_ptr<PFPacket>> packets;
  std::size_t used = 0;
  if (!parser.parse(bytes.data(), bytes.size(), packets, used, rclcpp::get_logger("packet_fixture")) ||
      packets.size() != 1 || used != bytes.size())
  {
    throw std::runtime_error("R2000 packet fixture did not parse exactly once");
  }
  return std::move(packets.front());
}

TEST(PacketHeader, ParsedR2000UsesItsDecodedHeaderThroughTheBaseClass)
{
  auto packet = parsedPacket(0xFEDC, 0x1234);
  EXPECT_EQ(packet->header.scan_number, 0u);
  EXPECT_EQ(packet->commonHeader().scan_number, 0xFEDCu);
  EXPECT_EQ(packet->commonHeader().packet_number, 0x1234u);
  const auto* concrete = dynamic_cast<const PFR2000Packet_C*>(packet.get());
  ASSERT_NE(concrete, nullptr);
  EXPECT_EQ(&packet->commonHeader(), &concrete->header.header);
}

TEST(PacketHeader, R2300HeaderDecoderExposesTheCommonFields)
{
  boost::shared_array<uint8_t> bytes(new uint8_t[256]{});
  bytes[10] = 0xCD;
  bytes[11] = 0xAB;
  bytes[12] = 0x34;
  bytes[13] = 0x12;
  PFR2300Packet_C1 packet;
  packet.read_header(bytes);
  const PFPacket& base = packet;
  EXPECT_EQ(base.header.scan_number, 0u);
  EXPECT_EQ(base.commonHeader().scan_number, 0xABCDu);
  EXPECT_EQ(base.commonHeader().packet_number, 0x1234u);
}

struct QueueSequence
{
  std::mutex mutex;
  std::condition_variable changed;
  bool entered = false;
  bool release_read = false;
  bool burst_processed = false;
  bool allow_end = false;
  bool stopped = false;
  bool timed_out = false;
  std::vector<std::pair<uint16_t, uint16_t>> consumed;

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex);
    stopped = true;
    release_read = true;
    allow_end = true;
    changed.notify_all();
  }
};

class PausedReader : public Reader<PFPacket>
{
public:
  explicit PausedReader(std::shared_ptr<QueueSequence> sequence) : sequence_(std::move(sequence))
  {
  }

  bool start() override
  {
    return true;
  }

  bool stop() override
  {
    // The production writer calls this before setting net_fail.
    sequence_->release();
    return true;
  }

  void read(std::shared_ptr<PFPacket> packet) override
  {
    std::unique_lock<std::mutex> lock(sequence_->mutex);
    sequence_->entered = true;
    sequence_->changed.notify_all();
    if (!sequence_->changed.wait_for(lock, 5s, [&] { return sequence_->release_read; }))
    {
      sequence_->timed_out = true;
    }
    const auto& header = packet->commonHeader();
    sequence_->consumed.emplace_back(header.scan_number, header.packet_number);
    sequence_->changed.notify_all();
  }

private:
  std::shared_ptr<QueueSequence> sequence_;
};

class BurstWriter : public Writer<PFPacket>
{
public:
  explicit BurstWriter(std::shared_ptr<QueueSequence> sequence) : sequence_(std::move(sequence))
  {
  }

  void prepare(std::size_t capacity, bool include_null = false)
  {
    include_null_ = include_null;
    for (std::size_t number = 1; number <= capacity + 6; ++number)
    {
      packets_.push_back(parsedPacket(37, static_cast<uint16_t>(number)));
    }
  }

  bool start() override
  {
    return true;
  }

  bool stop() override
  {
    sequence_->release();
    return true;
  }

  bool get(std::vector<std::unique_ptr<PFPacket>>& packets) override
  {
    if (phase_++ == 0)
    {
      if (include_null_)
      {
        packets.push_back(nullptr);
      }
      packets.push_back(std::move(packets_.front()));
      return true;
    }
    std::unique_lock<std::mutex> lock(sequence_->mutex);
    if (phase_ == 2)
    {
      if (!sequence_->changed.wait_for(lock, 5s, [&] { return sequence_->entered || sequence_->stopped; }))
      {
        sequence_->timed_out = true;
        return false;
      }
      if (sequence_->stopped)
      {
        return false;
      }
      for (std::size_t i = 1; i < packets_.size(); ++i)
      {
        packets.push_back(std::move(packets_[i]));
      }
      if (include_null_)
      {
        // This entry encounters the full queue but is not a packet loss.
        packets.push_back(nullptr);
      }
      return true;
    }
    // Reaching the next get proves the real enqueue loop finished the burst.
    sequence_->burst_processed = true;
    sequence_->changed.notify_all();
    if (!sequence_->changed.wait_for(lock, 5s, [&] { return sequence_->allow_end; }))
    {
      sequence_->timed_out = true;
    }
    return false;
  }

private:
  std::shared_ptr<QueueSequence> sequence_;
  std::vector<std::unique_ptr<PFPacket>> packets_;
  unsigned phase_ = 0;
  bool include_null_ = false;
};

class PipelineCleanup
{
public:
  PipelineCleanup(Pipeline& pipeline, std::shared_ptr<QueueSequence> sequence)
    : pipeline_(pipeline), sequence_(std::move(sequence))
  {
  }
  ~PipelineCleanup()
  {
    sequence_->release();
    pipeline_.terminate();
  }

private:
  Pipeline& pipeline_;
  std::shared_ptr<QueueSequence> sequence_;
};

TEST(PacketQueue, OverflowPreservesParsedIdsOrderTotalsAndNullEntries)
{
  std::mutex report_mutex;
  std::vector<DropReport> reports;
  auto counters = std::make_shared<DropCounters>([&](const DropReport& report) {
    std::lock_guard<std::mutex> lock(report_mutex);
    reports.push_back(report);
  });
  for (uint64_t restart = 0; restart < 2; ++restart)
  {
    auto sequence = std::make_shared<QueueSequence>();
    auto reader = std::make_shared<PausedReader>(sequence);
    auto writer = std::make_shared<BurstWriter>(sequence);
    auto net_mutex = std::make_shared<std::mutex>();
    auto net_cv = std::make_shared<std::condition_variable>();
    bool net_fail = false;
    Pipeline pipeline(writer, reader, [] {}, net_mutex, net_cv, net_fail, counters);
    const auto capacity = pipeline.queue_capacity();
    ASSERT_EQ(capacity, 127u);
    writer->prepare(capacity, restart != 0);
    PipelineCleanup cleanup(pipeline, sequence);
    ASSERT_TRUE(pipeline.start());
    {
      std::unique_lock<std::mutex> lock(sequence->mutex);
      ASSERT_TRUE(sequence->changed.wait_for(lock, 5s, [&] { return sequence->burst_processed; }));
      EXPECT_FALSE(sequence->timed_out);
      EXPECT_EQ(counters->snapshot().drops[2], 5u * (restart + 1));
      EXPECT_EQ(counters->snapshot().packet_queue_capacity, capacity);
      sequence->release_read = true;
      sequence->changed.notify_all();
      ASSERT_TRUE(sequence->changed.wait_for(lock, 5s, [&] { return sequence->consumed.size() == capacity + 1; }));
      for (std::size_t i = 0; i < sequence->consumed.size(); ++i)
      {
        EXPECT_EQ(sequence->consumed[i].first, 37u);
        EXPECT_EQ(sequence->consumed[i].second, i + 1);
      }
      sequence->allow_end = true;
      sequence->changed.notify_all();
    }
    {
      std::unique_lock<std::mutex> lock(*net_mutex);
      ASSERT_TRUE(net_cv->wait_for(lock, 5s, [&] { return net_fail; }));
    }
    pipeline.terminate();
    EXPECT_FALSE(pipeline.is_running());
    counters->tick();
    counters->tick();
  }
  std::lock_guard<std::mutex> lock(report_mutex);
  ASSERT_EQ(reports.size(), 4u);
  EXPECT_TRUE(reports[0].first);
  EXPECT_EQ(reports[0].details.scan_number, 37u);
  EXPECT_EQ(reports[0].details.packet_number, 129u);
  EXPECT_EQ(reports[0].details.queue_capacity, 127u);
  EXPECT_EQ(reports[1].count, 4u);
  EXPECT_TRUE(reports[2].first);
  EXPECT_EQ(reports[2].total, 6u);
  EXPECT_EQ(reports[3].count, 4u);
  EXPECT_EQ(reports[3].total, 10u);
}

TEST(PacketQueue, CleanupReleasesAnIntentionallyUnfinishedReader)
{
  auto sequence = std::make_shared<QueueSequence>();
  auto reader = std::make_shared<PausedReader>(sequence);
  auto writer = std::make_shared<BurstWriter>(sequence);
  auto net_mutex = std::make_shared<std::mutex>();
  auto net_cv = std::make_shared<std::condition_variable>();
  bool net_fail = false;
  Pipeline pipeline(writer, reader, [] {}, net_mutex, net_cv, net_fail);
  writer->prepare(pipeline.queue_capacity());
  {
    PipelineCleanup cleanup(pipeline, sequence);
    ASSERT_TRUE(pipeline.start());
    std::unique_lock<std::mutex> lock(sequence->mutex);
    ASSERT_TRUE(sequence->changed.wait_for(lock, 5s, [&] { return sequence->entered; }));
    // Leave the scope without advancing the normal release protocol.
  }
  EXPECT_FALSE(pipeline.is_running());
  std::lock_guard<std::mutex> lock(sequence->mutex);
  EXPECT_FALSE(sequence->timed_out);
}

}  // namespace
