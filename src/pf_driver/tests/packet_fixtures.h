#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "pf_driver/pf/pf_packet/pf_r2000_packet_c.h"
#include "pf_driver/pf/pf_packet/pf_r2300_packet_c1.h"
#include "pf_driver/ros/pf_data_publisher.h"

namespace pf_test
{

inline std::shared_ptr<ScanConfig> scanConfig()
{
  auto config = std::make_shared<ScanConfig>();
  config->timesync_method = TIMESYNC_METHOD_OFF;
  return config;
}

inline std::shared_ptr<ScanParameters> scanParameters()
{
  auto params = std::make_shared<ScanParameters>();
  params->radial_range_min = 0.1;
  params->radial_range_max = 50.0;
  params->apply_correction = false;
  return params;
}

template <typename Packet = PFR2000Packet_C>
Packet makePacket(uint16_t scan, uint16_t number, uint16_t points = 3600, uint16_t packet_points = 332)
{
  if (number == 0 || packet_points == 0 || (number - 1u) * packet_points >= points)
  {
    throw std::invalid_argument("packet number is outside this synthetic scan");
  }
  Packet packet;
  packet.header.header.scan_number = scan;
  packet.header.header.packet_number = number;
  packet.header.scan_frequency = 35000;
  packet.header.num_points_scan = points;
  packet.header.first_index = (number - 1u) * packet_points;
  packet.header.num_points_packet = std::min<uint16_t>(packet_points, points - packet.header.first_index);
  packet.header.angular_increment = 1000;
  packet.header.first_angle = -1800000 + packet.header.first_index * 1000;
  packet.header.timestamp_raw = (static_cast<uint64_t>(scan) + 10u) << 32;
  packet.last_acquired_point_stamp = rclcpp::Time(100000, 0, RCL_SYSTEM_TIME);
  for (uint32_t i = 0; i < packet.header.num_points_packet; ++i)
  {
    packet.distance.push_back(1000u + packet.header.first_index + i);
    packet.amplitude.push_back(100u + (packet.header.first_index + i) % 100u);
  }
  return packet;
}

struct PublishedScan
{
  uint16_t number;
  uint16_t layer;
  int inclination;
  sensor_msgs::msg::LaserScan message;
};

class RecordingPublisher : public PFDataPublisher
{
public:
  explicit RecordingPublisher(std::shared_ptr<ScanParameters> params = scanParameters(),
                              std::shared_ptr<DropCounters> counters = nullptr)
    : PFDataPublisher(scanConfig(), std::move(params), std::move(counters))
  {
    frame_id_ = "synthetic_laser";
  }

  std::vector<PublishedScan> published;

protected:
  void handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer, int inclination,
                   bool apply_correction) override
  {
    // The assembler can mutate msg again after this callback returns.
    published.push_back({ scan_number_, layer, inclination, *msg });
  }
};

inline void feedScan(RecordingPublisher& publisher, uint16_t scan, uint16_t omit = 0)
{
  for (uint16_t number = 1; number <= 11; ++number)
  {
    if (number != omit)
    {
      auto packet = makePacket(scan, number);
      publisher.read(packet);
    }
  }
}

inline std::vector<uint16_t> publishedNumbers(const RecordingPublisher& publisher)
{
  std::vector<uint16_t> result;
  for (const auto& scan : publisher.published)
  {
    result.push_back(scan.number);
  }
  return result;
}

}  // namespace pf_test
