#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "packet_fixtures.h"
#include "pf_driver/pf/drop_counters.h"

namespace
{

using namespace pf_test;

uint64_t count(const std::shared_ptr<DropCounters>& counters, DropReason reason)
{
  return counters->snapshot().drops.at(static_cast<std::size_t>(reason));
}

TEST(DropCounters, MissingFirstIsCountedOnceAndNamesTheDiscardedPacket)
{
  std::vector<DropReport> reports;
  auto counters = std::make_shared<DropCounters>([&](const DropReport& report) { reports.push_back(report); });
  RecordingPublisher publisher(scanParameters(), counters);
  for (uint16_t scan = 0; scan < 5; ++scan)
  {
    feedScan(publisher, scan, scan == 2 ? 1 : 0);
  }
  EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 1u);
  EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 0u);
  EXPECT_EQ(counters->snapshot().scans_published, 4u);
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_TRUE(reports.front().first);
  EXPECT_EQ(reports.front().details.scan_number, 2u);
  EXPECT_EQ(reports.front().details.packet_number, 2u);
}

TEST(DropCounters, MissingMiddleOrLastIsCountedOnlyWhenReplaced)
{
  for (const uint16_t omit : { 6, 11 })
  {
    auto counters = std::make_shared<DropCounters>();
    RecordingPublisher publisher(scanParameters(), counters);
    feedScan(publisher, 0);
    feedScan(publisher, 1, omit);
    EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 0u);
    feedScan(publisher, 2);
    EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 1u);
    EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 0u);
    EXPECT_EQ(counters->snapshot().scans_published, 2u);
  }
}

TEST(DropCounters, ConsecutiveReorderedStartsDoNotCountTheEarlierScanAgain)
{
  auto counters = std::make_shared<DropCounters>();
  RecordingPublisher publisher(scanParameters(), counters);
  feedScan(publisher, 0);
  for (uint16_t scan = 1; scan <= 2; ++scan)
  {
    auto packet = makePacket(scan, 2);
    publisher.read(packet);
    feedScan(publisher, scan, 2);
  }
  feedScan(publisher, 3);
  EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 2u);
  EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 0u);
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 3 }));
  EXPECT_EQ(counters->snapshot().scans_published, 2u);
}

TEST(DropCounters, InterleavedMissingStartsAreAttributedOncePerRetainedNumber)
{
  auto counters = std::make_shared<DropCounters>();
  RecordingPublisher publisher(scanParameters(), counters);
  feedScan(publisher, 0);
  for (const uint16_t scan : { 1, 2, 1, 3, 2, 3 })
  {
    auto packet = makePacket(scan, 2);
    publisher.read(packet);
  }
  feedScan(publisher, 4);
  EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 3u);
  EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 0u);
  EXPECT_EQ(counters->snapshot().scans_published, 2u);
}

TEST(DropCounters, APreviouslyAffectedScanCanStillBePublished)
{
  auto counters = std::make_shared<DropCounters>();
  RecordingPublisher publisher(scanParameters(), counters);
  feedScan(publisher, 0);
  auto early = makePacket(1, 2);
  publisher.read(early);
  feedScan(publisher, 1, 2);
  publisher.read(early);
  feedScan(publisher, 2);
  EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 1u);
  EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 0u);
  EXPECT_EQ(counters->snapshot().scans_published, 3u);
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 1, 2 }));
}

TEST(DropCounters, DifferentDropReasonsRemainSeparate)
{
  auto counters = std::make_shared<DropCounters>();
  RecordingPublisher publisher(scanParameters(), counters);
  feedScan(publisher, 0);
  feedScan(publisher, 1, 6);
  feedScan(publisher, 2, 1);
  feedScan(publisher, 3);
  EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 1u);
  EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 1u);
  EXPECT_EQ(count(counters, DropReason::QUEUE_FULL), 0u);
}

TEST(DropCounters, ActiveAttributionSurvivesRecentHistoryEviction)
{
  auto counters = std::make_shared<DropCounters>();
  RecordingPublisher publisher(scanParameters(), counters);
  feedScan(publisher, 0);
  auto early = makePacket(1, 2);
  publisher.read(early);
  feedScan(publisher, 1, 2);
  for (uint16_t scan = 2; scan < 258; ++scan)
  {
    auto packet = makePacket(scan, 2);
    publisher.read(packet);
  }
  feedScan(publisher, 258);
  EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 257u);
  EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 0u);
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 258 }));
}

TEST(DropCounters, HistoryEvictionPermitsANewAttributionWithoutRefreshingDuplicates)
{
  auto counters = std::make_shared<DropCounters>();
  RecordingPublisher publisher(scanParameters(), counters);
  feedScan(publisher, 0);
  for (uint16_t scan = 1; scan <= 256; ++scan)
  {
    auto packet = makePacket(scan, 2);
    publisher.read(packet);
  }
  auto first = makePacket(1, 2);
  publisher.read(first);
  EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 256u);
  auto next = makePacket(257, 2);
  publisher.read(next);
  publisher.read(first);
  EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 258u);
}

TEST(DropCounters, PublisherRecreationKeepsTotalsAndRestartsAttribution)
{
  auto counters = std::make_shared<DropCounters>();
  for (int restart = 0; restart < 2; ++restart)
  {
    RecordingPublisher publisher(scanParameters(), counters);
    auto startup = makePacket(9, 4);
    publisher.read(startup);
    feedScan(publisher, 10);
    feedScan(publisher, 11, 1);
    feedScan(publisher, 12, 11);
    publisher.stop();
  }
  EXPECT_EQ(count(counters, DropReason::MISSING_FIRST_PACKET), 2u);
  EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 0u);
  EXPECT_EQ(counters->snapshot().scans_published, 2u);
}

TEST(DropCounters, WrapAndR2300LayerCallsAreCountedWithoutChangingTheScans)
{
  auto counters = std::make_shared<DropCounters>();
  RecordingPublisher publisher(scanParameters(), counters);
  for (const uint16_t scan : { 65534, 65535, 0, 1 })
  {
    feedScan(publisher, scan);
  }
  for (uint16_t layer = 0; layer < 2; ++layer)
  {
    auto packet = makePacket<PFR2300Packet_C1>(2 + layer, 1, 4, 4);
    packet.header.layer_index = layer;
    publisher.read(packet);
  }
  EXPECT_EQ(counters->snapshot().scans_published, publisher.published.size());
  EXPECT_EQ(counters->snapshot().scans_published, 6u);
  EXPECT_EQ(counters->snapshot().drops, (std::array<uint64_t, 3>{ 0, 0, 0 }));
}

TEST(DropCounters, DuplicatePointPublicationDoesNotBecomeAnIncompleteDrop)
{
  auto counters = std::make_shared<DropCounters>();
  RecordingPublisher publisher(scanParameters(), counters);
  auto first = makePacket(1, 1, 12, 4);
  auto second = makePacket(1, 2, 12, 4);
  publisher.read(first);
  publisher.read(first);
  publisher.read(second);
  feedScan(publisher, 2);
  EXPECT_EQ(counters->snapshot().scans_published, 2u);
  EXPECT_EQ(count(counters, DropReason::INCOMPLETE_SCAN), 0u);
  ASSERT_EQ(publisher.published.size(), 2u);
  EXPECT_FLOAT_EQ(publisher.published.front().message.ranges[8], 0.0f);
}

TEST(DropSummary, FirstDropAndFinalSummaryAccountForTheWholeSeries)
{
  std::vector<DropReport> reports;
  DropCounters counters([&](const DropReport& report) { reports.push_back(report); });
  counters.recordDrop(DropReason::QUEUE_FULL);
  counters.recordDrop(DropReason::QUEUE_FULL);
  counters.recordDrop(DropReason::QUEUE_FULL);
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_EQ(reports.front().count, 1u);
  counters.tick();
  ASSERT_EQ(reports.size(), 2u);
  EXPECT_FALSE(reports.back().first);
  EXPECT_EQ(reports.back().count, 2u);
  EXPECT_EQ(reports.back().total, 3u);
  counters.tick();
  EXPECT_EQ(reports.size(), 2u);
  counters.recordDrop(DropReason::QUEUE_FULL);
  ASSERT_EQ(reports.size(), 3u);
  EXPECT_TRUE(reports.back().first);
  EXPECT_EQ(reports.back().total, 4u);
}

TEST(DropSummary, AnAlreadyReportedFirstDropDoesNotMakeTheIntervalQuiet)
{
  std::vector<DropReport> reports;
  DropCounters counters([&](const DropReport& report) { reports.push_back(report); });
  counters.recordDrop(DropReason::MISSING_FIRST_PACKET);
  counters.tick();
  counters.recordDrop(DropReason::MISSING_FIRST_PACKET);
  EXPECT_EQ(reports.size(), 1u);
  counters.tick();
  ASSERT_EQ(reports.size(), 2u);
  EXPECT_FALSE(reports.back().first);
  counters.recordDrop(DropReason::INCOMPLETE_SCAN);
  ASSERT_EQ(reports.size(), 3u);
  EXPECT_TRUE(reports.back().first);
  counters.tick();
  counters.recordDrop(DropReason::MISSING_FIRST_PACKET);
  ASSERT_EQ(reports.size(), 4u);
  EXPECT_TRUE(reports.back().first);
}

TEST(DropSummary, ConcurrentDropsPublicationsAndTicksPreserveTotals)
{
  std::array<std::atomic<uint64_t>, 3> reported{};
  std::atomic<bool> finished{ false };
  DropCounters* self = nullptr;
  DropCounters counters([&](const DropReport& report) {
    reported[static_cast<std::size_t>(report.reason)].fetch_add(report.count);
    // This also checks that caller code runs outside the accounting mutex.
    EXPECT_GE(self->snapshot().drops[static_cast<std::size_t>(report.reason)], report.total);
  });
  self = &counters;
  std::vector<std::thread> tickers;
  for (int i = 0; i < 2; ++i)
  {
    tickers.emplace_back([&] {
      while (!finished.load())
      {
        counters.tick();
        std::this_thread::yield();
      }
    });
  }
  std::vector<std::thread> writers;
  for (std::size_t reason = 0; reason < 3; ++reason)
  {
    writers.emplace_back([&, reason] {
      for (int i = 0; i < 10000; ++i)
      {
        counters.recordDrop(static_cast<DropReason>(reason));
        counters.recordPublication();
      }
    });
  }
  for (auto& writer : writers)
  {
    writer.join();
  }
  finished = true;
  for (auto& ticker : tickers)
  {
    ticker.join();
  }
  counters.tick();
  for (std::size_t reason = 0; reason < 3; ++reason)
  {
    EXPECT_EQ(reported[reason].load(), 10000u);
    EXPECT_EQ(counters.snapshot().drops[reason], 10000u);
  }
  EXPECT_EQ(counters.snapshot().scans_published, 30000u);
}

TEST(RecentScanAttribution, RetentionEvictionAndDuplicatesHaveExplicitBoundaries)
{
  RecentScanAttribution history;
  for (uint16_t scan = 0; scan < RecentScanAttribution::CAPACITY; ++scan)
  {
    ASSERT_TRUE(history.remember(scan));
  }
  EXPECT_TRUE(history.contains(0));
  EXPECT_FALSE(history.remember(0));
  EXPECT_TRUE(history.remember(256));
  EXPECT_FALSE(history.contains(0));
  EXPECT_TRUE(history.contains(1));
  EXPECT_TRUE(history.remember(0));
  EXPECT_FALSE(history.contains(1));
}

TEST(RecentScanAttribution, SequentialWrapsForgetThePreviousUseOfTheNumber)
{
  RecentScanAttribution history;
  for (uint32_t scan = 0; scan < 131100; ++scan)
  {
    ASSERT_TRUE(history.remember(static_cast<uint16_t>(scan))) << scan;
  }
}

}  // namespace
