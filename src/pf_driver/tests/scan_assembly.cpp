#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "packet_fixtures.h"

namespace
{

using namespace pf_test;

TEST(ScanAssembly, CompleteScansPreserveGeometryTimingAndValues)
{
  RecordingPublisher publisher;
  for (uint16_t scan = 0; scan < 5; ++scan)
  {
    feedScan(publisher, scan);
  }
  ASSERT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 1, 2, 3, 4 }));
  for (const auto& scan : publisher.published)
  {
    const auto& msg = scan.message;
    EXPECT_EQ(msg.header.frame_id, "synthetic_laser");
    EXPECT_EQ(msg.header.stamp.sec, scan.number + 10);
    EXPECT_EQ(msg.header.stamp.nanosec, 0u);
    EXPECT_FLOAT_EQ(msg.scan_time, 0.028f);
    EXPECT_FLOAT_EQ(msg.time_increment, static_cast<float>(0.028 * 1000.0 / 3600000.0));
    EXPECT_FLOAT_EQ(msg.angle_min, static_cast<float>(-M_PI));
    EXPECT_FLOAT_EQ(msg.angle_increment, static_cast<float>(0.1 * M_PI / 180.0));
    EXPECT_FLOAT_EQ(msg.angle_max, static_cast<float>(M_PI));
    EXPECT_FLOAT_EQ(msg.range_min, 0.1f);
    EXPECT_FLOAT_EQ(msg.range_max, 50.0f);
    ASSERT_EQ(msg.ranges.size(), 3600u);
    ASSERT_EQ(msg.intensities.size(), 3600u);
    for (std::size_t i = 0; i < msg.ranges.size(); ++i)
    {
      EXPECT_FLOAT_EQ(msg.ranges[i], static_cast<float>((1000.0 + i) / 1000.0));
      EXPECT_FLOAT_EQ(msg.intensities[i], static_cast<float>(100u + i % 100u));
    }
  }
}

TEST(ScanAssembly, MissingFirstMiddleOrLastPacketDoesNotPublishThatScan)
{
  for (const uint16_t omit : { 1, 6, 11 })
  {
    SCOPED_TRACE(omit);
    RecordingPublisher publisher;
    for (uint16_t scan = 0; scan < 5; ++scan)
    {
      feedScan(publisher, scan, scan == 2 ? omit : 0);
    }
    EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 1, 3, 4 }));
  }
}

TEST(ScanAssembly, ReorderedStartWithoutRedeliveryStaysIncomplete)
{
  RecordingPublisher publisher;
  feedScan(publisher, 0);
  auto early = makePacket(1, 2);
  publisher.read(early);
  feedScan(publisher, 1, 2);
  feedScan(publisher, 2);
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 2 }));
}

TEST(ScanAssembly, StartupPartialAndConsecutiveMissingStartsAreNotPublished)
{
  RecordingPublisher publisher;
  for (uint16_t number = 4; number <= 11; ++number)
  {
    auto packet = makePacket(0, number);
    publisher.read(packet);
  }
  feedScan(publisher, 1);
  for (uint16_t scan = 2; scan <= 4; ++scan)
  {
    feedScan(publisher, scan, 1);
  }
  feedScan(publisher, 5);
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 1, 5 }));
}

TEST(ScanAssembly, MissingMiddleThenMissingStartPreservesTheNextCompleteScan)
{
  RecordingPublisher publisher;
  feedScan(publisher, 0);
  feedScan(publisher, 1);
  feedScan(publisher, 2, 6);
  feedScan(publisher, 3, 1);
  feedScan(publisher, 4);
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 1, 4 }));
}

TEST(ScanAssembly, ConsecutiveReorderedStartsPreserveAssembly)
{
  RecordingPublisher publisher;
  feedScan(publisher, 0);
  for (uint16_t scan = 1; scan <= 2; ++scan)
  {
    auto early = makePacket(scan, 2);
    publisher.read(early);
    feedScan(publisher, scan, 2);
  }
  feedScan(publisher, 3);
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 3 }));
}

TEST(ScanAssembly, RedeliveredEarlyPacketCanRestorePublication)
{
  RecordingPublisher publisher;
  feedScan(publisher, 0);
  auto early = makePacket(1, 2);
  publisher.read(early);
  feedScan(publisher, 1, 2);
  ASSERT_EQ(publisher.published.size(), 1u);
  publisher.read(early);
  feedScan(publisher, 2);
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 1, 2 }));
}

TEST(ScanAssembly, ScanNumberWrapKeepsPublicationOrder)
{
  RecordingPublisher publisher;
  for (const uint16_t scan : { 65534, 65535, 0, 1 })
  {
    feedScan(publisher, scan);
  }
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 65534, 65535, 0, 1 }));
}

TEST(ScanAssembly, DuplicatePointTotalCanPublishBeforeUniqueCoverage)
{
  RecordingPublisher publisher;
  auto first = makePacket(1, 1, 12, 4);
  auto second = makePacket(1, 2, 12, 4);
  auto last = makePacket(1, 3, 12, 4);
  publisher.read(first);
  publisher.read(first);
  publisher.read(second);
  ASSERT_EQ(publisher.published.size(), 1u);
  ASSERT_EQ(publisher.published.front().message.ranges.size(), 12u);
  EXPECT_FLOAT_EQ(publisher.published.front().message.ranges[8], 0.0f);
  publisher.read(last);
  // Characterize inherited point-sum behavior and the fixture's deep copy.
  EXPECT_EQ(publisher.published.size(), 1u);
  EXPECT_FLOAT_EQ(publisher.published.front().message.ranges[8], 0.0f);
}

TEST(ScanAssembly, DuplicateAfterPublicationDoesNotUndoThePublishedResult)
{
  RecordingPublisher publisher;
  feedScan(publisher, 0);
  auto duplicate = makePacket(0, 2);
  duplicate.distance.assign(duplicate.distance.size(), 47000u);
  publisher.read(duplicate);
  feedScan(publisher, 1);
  ASSERT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0, 1 }));
  EXPECT_FLOAT_EQ(publisher.published.front().message.ranges[332], 1.332f);
}

TEST(ScanAssembly, IncompleteTerminationTailIsNotPublished)
{
  RecordingPublisher publisher;
  feedScan(publisher, 0);
  feedScan(publisher, 1, 11);
  publisher.stop();
  EXPECT_EQ(publishedNumbers(publisher), (std::vector<uint16_t>{ 0 }));
}

TEST(ScanAssembly, R2300ReportsEachCompletedLayerCallback)
{
  auto params = scanParameters();
  params->scan_time_factor = 2;
  RecordingPublisher publisher(params);
  for (uint16_t layer = 0; layer < 2; ++layer)
  {
    auto packet = makePacket<PFR2300Packet_C1>(20 + layer, 1, 4, 4);
    packet.header.layer_index = layer;
    packet.header.layer_inclination = 10000 * layer;
    publisher.read(packet);
  }
  ASSERT_EQ(publisher.published.size(), 2u);
  for (std::size_t i = 0; i < publisher.published.size(); ++i)
  {
    EXPECT_EQ(publisher.published[i].layer, i);
    EXPECT_EQ(publisher.published[i].inclination, 10000 * i);
    EXPECT_FLOAT_EQ(publisher.published[i].message.scan_time, 0.056f);
    EXPECT_FLOAT_EQ(publisher.published[i].message.time_increment, 1.0f / 252000.0f);
  }
}

}  // namespace
