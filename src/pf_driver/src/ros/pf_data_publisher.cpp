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

/** Update statistics for timesync based on packet reception time,
 *  given a packet with sensor time and packet reception time. The
 *  time_increment (sample period) must be known and passed in msg.
 */
template <typename T>
void PFDataPublisher::update_timesync(T& packet)
{
  /* The timestamp in the packet tells about the time when the first sample in the
     packet was measured, but now is rather shortly after the *last* sample in the
     packet was measured (plus transmission and OS processing time that we don't know). */

  /* To compute this, the time_increment must be known and up to date */
  const auto time_for_points_in_packet =
      rclcpp::Duration(0, packet.header.num_points_packet * (unsigned int)(1.0E9 * msg_->time_increment));
  params_->passive_timesync.update(packet.header.timestamp_raw, 0, rclcpp::Clock().now() - time_for_points_in_packet);

  RCLCPP_DEBUG(rclcpp::get_logger("timesync"), "packet#%d.%d.%d with %x %lu %.3f %u %u %f", scan_number_,
               packet.header.header.scan_number, packet.header.header.packet_number, packet.header.status_flags,
               packet.header.timestamp_raw, (rclcpp::Clock().now() - time_for_points_in_packet).seconds(),
               packet.header.num_points_packet, packet.header.scan_frequency, 1.0E6 * msg_->time_increment);
}

// What are validation checks required here?
// Skipped scans?
// Device errors?
template <typename T>
void PFDataPublisher::to_msg_queue(T& packet, uint16_t layer_idx, int layer_inclination)
{
  if (packet.header.status_flags != 0)
  {
    /* Information in packet is inconsistent.

      Most probably (at startup or after changing parameters) with R2000 the
      header.scan_frequency does not match the actual physical frequency ("unstable
      rotation") and thus the time_increment from header does not match the
      real sample frequency. TBD: Just ignore this packet or all up to now?
    */
    params_->passive_timesync.reset(0.0);
  }

  if (!!msg_ && (packet.header.header.scan_number == scan_number_))
  {
    /* msg_ already initialized and packet belongs to same scan, ie. is not first packet:
        No need to update msg_.header details. Just update passive timesync from packet. */

    update_timesync(packet);
  }
  else
  {
    /* The incoming packet belongs to another scan than we have been recording until now
        (or, if !msg, it is the very first packet ever since we started receiving) */

    /* TBD: Check if msg already has some data and handle_scan even if it's incomplete? */
    //  handle_scan(msg_, layer_idx, layer_inclination, params_->apply_correction);

    if (packet.header.header.packet_number != 1)
    {
      /* TBD: Discard whole scan if any packet is missing? */
      // return;
    }

    msg_.reset(new sensor_msgs::msg::LaserScan());

    msg_->header.frame_id.assign(frame_id_);
    scan_number_ = packet.header.header.scan_number;

    const auto scan_time = rclcpp::Duration(0, 1000000ul * (1000000ul / packet.header.scan_frequency));
    msg_->scan_time = static_cast<float>(scan_time.seconds());

    msg_->angle_increment = packet.header.angular_increment / 10000.0 * (M_PI / 180.0);

    {
      /* Assuming that angle_min always means *first* angle (which may be numerically
       * greater than angle_max in case of negative angular_increment during CW rotation) */

      msg_->angle_min = ((double)packet.header.first_angle) * (M_PI / 1800000.0);
      msg_->angle_max = msg_->angle_min +
                        ((double)packet.header.num_points_scan * packet.header.angular_increment) * (M_PI / 1800000.0);

      if (std::is_same<T, PFR2300Packet_C1>::value)  // packet interpretation specific to R2300 output
      {
        /* If scans are output on separate topics per layer, the time between messages per topic grows by the number of
         * layers */
        if (params_->scan_time_factor > 1)
        {
          msg_->scan_time *= (float)(params_->scan_time_factor);
        }

        double orig_angular_increment = 0.1 * (M_PI / 180.0);
        if (packet.header.scan_frequency > 50000)
        {
          orig_angular_increment = 0.2 * (M_PI / 180.0);
        };
        /* Consider effective longer time_increment due to filtering with decimation */
        double decimation = round(orig_angular_increment / msg_->angle_increment);

        /* Assuming that sampling_rate_max==sampling_rate_min==const. */
        msg_->time_increment = decimation / (double)(params_->sampling_rate_max);
      }
      else
      {
        msg_->time_increment = fabs(scan_time.seconds() * (double)packet.header.angular_increment * (1.0 / 3600000.0));
      }

      msg_->range_min = params_->radial_range_min;
      msg_->range_max = params_->radial_range_max;
    }

    update_timesync(packet);

    rclcpp::Time first_acquired_point_stamp;
    if (config_->timesync_method == TIMESYNC_METHOD_REQUESTS && params_->active_timesync.valid())
    {
      params_->active_timesync.sensor_to_pc(packet.header.timestamp_raw, first_acquired_point_stamp);
    }
    else if (config_->timesync_method == TIMESYNC_METHOD_AVERAGE && params_->passive_timesync.valid())
    {
      params_->passive_timesync.sensor_to_pc(packet.header.timestamp_raw, first_acquired_point_stamp);
    }
    else if (config_->timesync_method == TIMESYNC_METHOD_SIMPLE)
    {
      /* No averaging, just estimate acquisition time from most recent reception time */
      int points_to_end_of_packet = packet.header.first_index + packet.header.num_points_packet;
      const auto time_for_measurement =
          rclcpp::Duration(0, points_to_end_of_packet * (unsigned int)(1.0E9 * msg_->time_increment) +
                                  1000 * config_->timesync_offset_usec);

      first_acquired_point_stamp = packet.last_acquired_point_stamp - time_for_measurement;
    }
    else /* config_->timesync_method == TIMESYNC_METHOD_OFF) */
    {
      /* Hopefully the timestamp_raw never exceeds 0x7fffffff.ffffffff seconds */
      int32_t seconds = (int32_t)((packet.header.timestamp_raw >> 32) & 0xfffffffful);
      uint32_t nanoseconds = ((packet.header.timestamp_raw & 0xffffffffull) * 1000000000ull) >> 32;

      /* TBD: Construct rclcpp::Time with same clock_type as last_acquired_point_stamp for consistency? */
      first_acquired_point_stamp =
          rclcpp::Time(seconds, nanoseconds, packet.last_acquired_point_stamp.get_clock_type());
    }
    msg_->header.stamp = first_acquired_point_stamp;

    msg_->ranges.resize(packet.header.num_points_scan);
    msg_->intensities.resize(packet.amplitude.empty() ? 0 : packet.header.num_points_scan);

    /* TBD: Preload ranges and intensities with NaN? */
    // msg_->ranges.assign(packet.header.num_points_scan, vector<float>(size, std::numeric_limits<float>::quiet_NaN()));
  }

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
      msg_->intensities[idx + i] = std::move(echo);
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
    msg_->ranges[idx + i] = std::move(data);
  }

  if (packet.header.num_points_scan == (idx + packet.header.num_points_packet))
  {
    handle_scan(msg_, layer_idx, layer_inclination, params_->apply_correction);
  }
}
