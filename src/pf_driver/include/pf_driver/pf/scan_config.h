#pragma once

#include <string>

struct ScanConfig
{
  bool watchdog = false;
  bool watchdog_set = false;

  uint watchdogtimeout = 0;
  bool watchdogtimeout_set = false;

  std::string packet_type = "";
  bool packet_type_set = false;

  int start_angle = 0;
  bool start_angle_set = false;

  uint max_num_points_scan = 0;
  bool max_num_points_scan_set = false;

  uint skip_scans = 0;
  bool skip_scans_set = false;

  // void print()
  // {
  //   std::cout << "Scan output config:\n"
  //             << "watchdogtimeout: " << watchdogtimeout << "\n"
  //             << "packet_type: " << packet_type << "\n"
  //             << "start_angle: " << start_angle << "\n"
  //             << "max_num_points_scan:" << max_num_points_scan << "\n"
  //             << "skip_scan: " << skip_scans << std::endl;
  // }
};
