## ROS Topics  

**R2000 & R2300 1-layer:**

| Topic         | Type          | Description                                                                                                                  |
| ------------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| /pf/scan      | LaserScan     | ROS standard message, see [sensor_msgs/LaserScan](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/LaserScan.html) |
| /r2000_header | PFR2000Header | Sensor specific message, see [pf_interfaces/msg/PFR2000Header.msg](../src/pf_interfaces/msg/PFR2000Header.msg)               |
| /diagnostics  | DiagnosticArray | Cumulative scan and packet drop counts; see [drop diagnostics](#drop-diagnostics). |

Topic PFR2000Header details:

| Type   | Name              | Description                                                                                                               |
| ------ | ----------------- | ------------------------------------------------------------------------------------------------------------------------- |
| uint16 | magic             | magic byte (0xa25c) marking the beginning of a packet                                                                     |
| uint16 | packet_type       | scan data packet type (0x0041 - 'A' 0x0042 - 'B' 0x0043 - 'C')                                                            |
| uint32 | packet_size       | overall packet size in bytes (header + payload)                                                                           |
| uint16 | header_size       | header size in bytes (offset to payload data)                                                                             |
| uint16 | scan_number       | sequence number for scan (incremented for every scan, starting with 0, overflows)                                         |
| uint16 | packet_number     | sequence number for packet (counting packets of a particular scan, starting with 1)                                       |
|        |                   |                                                                                                                           |
| uint64 | timestamp_raw     | raw timestamp of first scan point in this packet in NTP time format                                                       |
| uint64 | timestamp_sync    | synchronized timestamp of first scan point in this packet in NTP time format (currenty not available and and set to zero) |
| uint32 | status_flags      | scan status flags                                                                                                         |
| uint32 | scan_frequency    | frequency of head rotation  (1/1000Hz)                                                                                    |
| uint16 | num_points_scan   | number of scan points (samples) within complete scan                                                                      |
| uint16 | num_points_packet | total number of scan points within this packet                                                                            |
| uint16 | first_index       | index of first scan point within this packet                                                                              |
|  int32 | first_angle       | absolute angle of first scan point within this packet  (1/10000°)                                                         |
|  int32 | angular_increment | delta between two succeding scan points (1/10000°) CCW rotation: +ve, CW rotation: -ve                                    |
| uint32 | iq_input          | reserved - all bits zero for devices without switching I/Q                                                                |
| uint32 | iq_overload       | reserved - all bits zero for devices without switching I/Q                                                                |
| uint64 | iq_timestamp_raw  | raw timestamp for status of switching I/Q                                                                                 |
| uint64 | iq_timestamp_sync | synchronized timestamp for status of switching I/Q                                                                        |

status_flags details:

| Bit             | Flag name                  | Description                                                                       |
| --------------- |--------------------------- | --------------------------------------------------------------------------------- |
| *Informational* |                            |                                                                                   |
|  0              | scan_data_info             | Accumulative flag – set if any informational flag (bits 1..7) is set              |
|  1              | new_settings               | System settings for scan data acquisition changed during recording of this packet |
|  2              | invalid_data               | Consistency of scan data is not guaranteed for this packet                        |
|  3              | unstable_rotation          | Scan frequency did not match set value while recording this scan data packet      |
|  4              | skipped_packets            | Preceding scan data packets have been skipped due to connection issues            |
| *Warnings*      |                            |                                                                                   |
|  8              | device_warning             | Accumulative flag – set if any warning flag (bits 9..15) is set                   |
|  9              | lens_contamination_warning | LCM warning threshold triggered for at least one sector                           |
| 10              | low_temperature_warning    | Current internal temperature below warning threshold                              |
| 11              | high_temperature_warning   | Current internal temperature above warning threshold                              |
| 12              | device_overload            | Overload warning – sensor CPU overload is imminent                                |
| *Errors*        |                            |                                                                                   |
| 16              | device_error               | Accumulative flag – set if any error flag (bits 17..23) is set                    |
| 17              | lens_contamination_error   | LCM error threshold triggered for at least one sector                             |
| 18              | low_temperature_error      | Current internal temperature below error threshold                                |
| 19              | high_temperature_error     | Current internal temperature above error threshold                                |
| 20              | device_overload            | Overload error – sensor CPU is in overload state                                  |
| *Defects*       |                            |                                                                                   |
| 30              | device_defect              | Accumulative flag – set if device detected an unrecoverable defect                |

**R2300 4-layer:**

| Topic         | Type          | Description                                                                                                                      |
| ------------- |-------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| /pf/cloud     | PointCloud2   | ROS standard message, see [sensor_msgs/PointCloud2](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/PointCloud2.html) |
| /r2300_header | PFR2300Header | Sensor specific message, see [pf_interfaces/msg/PFR2300Header.msg](../src/pf_interfaces/msg/PFR2300Header.msg)                   |
| /diagnostics  | DiagnosticArray | Cumulative scan and packet drop counts; see [drop diagnostics](#drop-diagnostics). |

Topic PFR2300Header details:

| Type   | Name              | Description                                                                            |
| ------ | ----------------- | -------------------------------------------------------------------------------------- |
| uint16 | magic             | magic byte (0xa25c) marking the beginning of a packet                                  |
| uint16 | packet_type       | scan data packet type (0x0041 - 'A' 0x0042 - 'B' 0x0043 - 'C')                         |
| uint32 | packet_size       | overall packet size in bytes (header + payload)                                        |
| uint16 | header_size       | header size in bytes (offset to payload data)                                          |
| uint16 | scan_number       | sequence number for scan (incremented for every scan, starting with 0, overflows)      |
| uint16 | packet_number     | sequence number for packet (counting packets of a particular scan, starting with 1)    |
|        |                   |                                                                                        |
| uint16 | layer_index       | vertical layer index (0..3)                                                            |
|  int32 | layer_inclination | vertical layer inclination [1/10000 degree]                                            |
| uint64 | timestamp_raw     | raw timestamp of first scan point in this packet in NTP time format                    |
| uint64 | reserved1         | reserved - all bits zero for devices without switching I/Q                             |
| uint32 | status_flags      | scan status flags                                                                      |
| uint32 | scan_frequency    | frequency of head rotation  (1/1000Hz)                                                 |
| uint16 | num_points_scan   | number of scan points (samples) within complete scan                                   |
| uint16 | num_points_packet | total number of scan points within this packet                                         |
| uint16 | first_index       | index of first scan point within this packet                                           |
|  int32 | first_angle       | absolute angle of first scan point within this packet  (1/10000°)                      |
|  int32 | angular_increment | delta between two succeding scan points (1/10000°) CCW rotation: +ve, CW rotation: -ve |

status_flags details:

| Bit             | Flag name                  | Description                                                                       |
| --------------- |--------------------------- | --------------------------------------------------------------------------------- |
| *Informational* |                            |                                                                                   |
|  0              | scan_data_info             | Accumulative flag – set if any informational flag (bits 1..7) is set              |
|  1              | new_settings               | System settings for scan data acquisition changed during recording of this packet |
|  2              | invalid_data               | Consistency of scan data is not guaranteed for this packet                        |
|  3              | unstable_rotation          | Scan frequency did not match set value while recording this scan data packet      |
|  4              | skipped_packets            | Preceding scan data packets have been skipped due to connection issues            |
| *Warnings*      |                            |                                                                                   |
|  8              | device_warning             | Accumulative flag – set if any warning flag (bits 9..15) is set                   |
| 10              | low_temperature_warning    | Current internal temperature below warning threshold                              |
| 11              | high_temperature_warning   | Current internal temperature above warning threshold                              |
| 12              | device_overload            | Overload warning – sensor CPU overload is imminent                                |
| *Errors*        |                            |                                                                                   |
| 16              | device_error               | Accumulative flag – set if any error flag (bits 17..23) is set                    |
| 18              | low_temperature_error      | Current internal temperature below error threshold                                |
| 19              | high_temperature_error     | Current internal temperature above error threshold                                |
| 20              | device_overload            | Overload error – sensor CPU is in overload state                                  |
| *Defects*       |                            |                                                                                   |
| 30              | device_defect              | Accumulative flag – set if device detected an unrecoverable defect                |

## Drop diagnostics

`/diagnostics` contains one `diagnostic_msgs/DiagnosticStatus` named
`<node name>: dropped scans and packets`. Four values are decimal integer counters
since this PFInterface was created, normally when the node starts. Recreating a
publisher or packet pipeline does not reset them. The fifth value is the current
packet queue capacity, not an accumulated count.

| Key | Meaning |
| --- | --- |
| `scans_published` | Calls to the scan handler, including individual R2300 layer calls. This does not count subscriber receipts or completed multilayer point clouds. |
| `scans_dropped_missing_first_packet` | Scan numbers attributed when a packet is discarded because that scan did not start with packet 1. Startup partials are excluded. |
| `scans_dropped_incomplete` | Incomplete scan assemblies discarded when another scan starts, excluding assemblies already attributed to a drop or passed to the handler. |
| `packets_dropped_queue_full` | Nonnull packets rejected by the real packet queue. |
| `packet_queue_capacity` | Actual capacity reported by the packet queue, currently 127. It is zero before the first pipeline is constructed. |

### Attribution limits

The publisher remembers the most recent 256 distinct scan numbers that it either
attributed to a drop or passed to the scan handler. Repeated observations do not
refresh a number's position in that history. It separately remembers whether the
current assembly was already attributed, even if interleaved packet arrivals
evict that number from the history. Thus two consecutive reordered starts do not
count the first affected scan again when its incomplete assembly is replaced.

Scan numbers are 16-bit values, not globally unique identities. After a number
leaves the history it can be counted again. Sequential rollover from 65535 to 0
works because the previous use of a number has left the history. Arbitrarily
delayed or reordered packets cannot be identified across an entire rollover.
Publisher recreation restarts this attribution history while preserving the
cumulative counters.

A discarded packet can arrive again after packet 1 and complete its scan. That
scan remains counted under its first attributed reason and can also increment
`scans_published`. Counting does not change packet acceptance or prevent that
publication. The scan counters therefore describe scans affected by observed
drops; they do not necessarily describe scans that were never published.

Packets arriving before the first scan has started with packet 1 are not counted
as missing-first drops. An incomplete final assembly is counted only if a later
scan replaces it; stopping or recreating the publisher does not count that tail.
These exclusions also apply when a stream starts after reconnecting.

The patch preserves the upstream received-point-sum publication rule. Duplicate
packets can make that sum reach the scan size before the distinct beam positions
are covered, or make it overshoot the size. The counters do not repair that
assembly behavior. Scan periods, beam timestamps, angles and point values keep
the upstream calculations, including the 0.028-second scan period at 35 Hz.

### Publication and logging

The read-only `diagnostics.period` parameter is a wall-clock period in seconds,
default `1.0`. It must be finite, positive and representable in nanoseconds.
The message header uses the node's ROS clock; with paused simulated time its
stamp can remain zero while diagnostic messages continue arriving. The hardware
ID initially contains `scanner_ip`, then the product and IP after device
initialization. Metadata is copied under a mutex.

The status is WARN when a drop counter increased since the previous diagnostic
update and OK otherwise. The four counters and current capacity are present in either case.
An OK status with unchanged publication and drop counts does not establish that
the device is connected or healthy; the diagnostic does not enforce an expected
scan rate.

The first drop of an armed reason logs a WARN line immediately. Further drops of
that reason are included in the next one-second wall-timer summary, without
repeating the first drop. A complete timer interval without a drop re-arms that
reason. The timer reports the remainder of a series even after input stops.
The summary timer is independent of `diagnostics.period`.

Accounting is synchronized across packet, publication and timer threads. Logging
occurs after releasing the accounting mutex, so concurrent report lines can be
written in a different order from the counter updates. Their counts still
partition the reported drops when logging and the executor keep running. The
diagnostics and summary timers share a dedicated mutually exclusive callback
group rather than the driver's default group used by device operations.




