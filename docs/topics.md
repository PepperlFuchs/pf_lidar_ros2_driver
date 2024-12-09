## ROS Topics  

**R2000 & R2300 1-layer:**

| Topic         | Type          | Description                                                                                                                  |
| ------------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| /pf/scan      | LaserScan     | ROS standard message, see [sensor_msgs/LaserScan](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/LaserScan.html) |
| /r2000_header | PFR2000Header | Sensor specific message, see [pf_interfaces/msg/PFR2000Header.msg](../src/pf_interfaces/msg/PFR2000Header.msg)               |

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






