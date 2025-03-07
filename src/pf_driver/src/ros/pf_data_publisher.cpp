// #include <exception>
// #include <limits>
// #include <utility>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include "pf_driver/ros/pf_data_publisher.h"
#include "pf_driver/pf/pf_packet/pf_r2000_packet_a.h"
#include "pf_driver/pf/pf_packet/pf_r2000_packet_b.h"
#include "pf_driver/pf/pf_packet/pf_r2000_packet_c.h"
#include "pf_driver/pf/pf_packet/pf_r2300_packet_c1.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PFDataPublisher::PFDataPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params)
  : config_(config), params_(params)
{
}

void PFDataPublisher::read(PFR2000Packet_A& packet)
{
  publish_header(packet.header);
  to_msg_queue<PFR2000Packet_A>(packet);
}

void PFDataPublisher::read(PFR2000Packet_B& packet)
{
  publish_header(packet.header);
  to_msg_queue<PFR2000Packet_B>(packet);
}

void PFDataPublisher::read(PFR2000Packet_C& packet)
{
  publish_header(packet.header);
  to_msg_queue<PFR2000Packet_C>(packet);
}

void PFDataPublisher::read(PFR2300Packet_C1& packet)
{
  publish_header(packet.header);
  to_msg_queue<PFR2300Packet_C1>(packet, packet.header.layer_index, packet.header.layer_inclination);
}

bool PFDataPublisher::start()
{
  return true;
}

bool PFDataPublisher::stop()
{
  return true;
}

// What are validation checks required here?
// Skipped scans?
// Device errors?
template <typename T>
void PFDataPublisher::to_msg_queue(T& packet, uint16_t layer_idx, int layer_inclination)
{
  if (!check_status(packet.header.status_flags))
    return;

  sensor_msgs::msg::LaserScan::SharedPtr msg;
  if (d_queue_.empty())
    d_queue_.emplace_back();
  else if (d_queue_.size() > 5)
    d_queue_.pop_front();
  if (packet.header.header.packet_number == 1)
  {
    const auto scan_time = rclcpp::Duration(0, 1000000ul * (1000000ul / packet.header.scan_frequency));
    msg.reset(new sensor_msgs::msg::LaserScan());
    msg->header.frame_id.assign(frame_id_);
    // msg->header.seq = packet.header.header.scan_number;
    msg->scan_time = static_cast<float>(scan_time.seconds());
    msg->header.stamp = packet.last_acquired_point_stamp - scan_time;
    msg->angle_increment = packet.header.angular_increment / 10000.0 * (M_PI / 180.0);

    {
      /* Assuming that angle_min always means *first* angle (which may be numerically
       * greater than angle_max in case of negative angular_increment during CW rotation) */

      msg->angle_min = ((double)packet.header.first_angle) * (M_PI / 1800000.0);
      msg->angle_max = msg->angle_min +
                       ((double)packet.header.num_points_scan * packet.header.angular_increment) * (M_PI / 1800000.0);

      if (std::is_same<T, PFR2300Packet_C1>::value)  // packet interpretation specific to R2300 output
      {
        /* If scans are output on separate topics per layer, the time between messages per topic grows by the number of
         * layers */
        if (params_->scan_time_factor > 1)
        {
          msg->scan_time *= (float)(params_->scan_time_factor);
        }

        double orig_angular_increment = 0.1 * (M_PI / 180.0);
        if (packet.header.scan_frequency > 50000)
        {
          orig_angular_increment = 0.2 * (M_PI / 180.0);
        };
        /* Consider effective longer time_increment due to filtering with decimation */
        double decimation = round(orig_angular_increment / msg->angle_increment);

        /* Assuming that sampling_rate_max==sampling_rate_min==const. */
        msg->time_increment = decimation / (double)(params_->sampling_rate_max);
      }
      else
      {
        msg->time_increment = fabs(scan_time.seconds() * (double)packet.header.angular_increment * (1.0 / 3600000.0));
      }

      msg->range_min = params_->radial_range_min;
      msg->range_max = params_->radial_range_max;
    }

    msg->ranges.resize(packet.header.num_points_scan);
    msg->intensities.resize(packet.amplitude.empty() ? 0 : packet.header.num_points_scan);

    d_queue_.push_back(msg);
  }
  msg = d_queue_.back();
  if (!msg)
    return;

  // errors in scan_number - not in sequence sometimes
  /*if (msg->header.seq != packet.header.header.scan_number)
    return;*/
  int idx = packet.header.first_index;

  for (int i = 0; i < packet.header.num_points_packet; i++)
  {
    float data;

    if (!packet.amplitude.empty())  // amplitude<32 indicates invalid measurement
    {
      float echo;
      if (packet.amplitude[i] >= 32)
      {
        data = packet.distance[i] / 1000.0;
        echo = packet.amplitude[i];
      }
      else if (packet.amplitude[i] == 0 || packet.amplitude[i] == 6)  // no or weak echo
      {
        data = std::numeric_limits<float>::infinity();
        echo = std::numeric_limits<float>::quiet_NaN();
      }
      else  // invalid measurement due to some other reason (TBD: blinding => -Inf?)
      {
        data = std::numeric_limits<float>::quiet_NaN();
        echo = std::numeric_limits<float>::quiet_NaN();
      }
      msg->intensities[idx + i] = std::move(echo);
    }
    else  // no amplitude came with packet, invalid values are encoded as all bits set
    {
      uint32_t distance_max =  // all bits set in 32 (A/B) or 20 bit (C/C1) => invalid measurement
          ((std::is_same<T, PFR2000Packet_A>::value) || (std::is_same<T, PFR2000Packet_B>::value)) ? 0xffffffffu :
                                                                                                     0x000fffffu;

      if (packet.distance[i] < distance_max)
      {
        data = packet.distance[i] / 1000.0;
      }
      else
      {
        data = std::numeric_limits<float>::quiet_NaN();
      }
    }
    msg->ranges[idx + i] = std::move(data);
  }

  if (packet.header.num_points_scan == (idx + packet.header.num_points_packet))
  {
    if (msg)
    {
      handle_scan(msg, layer_idx, layer_inclination, params_->apply_correction);
      d_queue_.pop_back();
    }
  }
}

// check the status bits here with a switch-case
// Currently only for logging purposes only
bool PFDataPublisher::check_status(uint32_t status_flags)
{
  // if(packet.header.header.scan_number > packet.)
  return true;
}
