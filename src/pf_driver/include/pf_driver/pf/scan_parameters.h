#pragma once

#include "pf_driver/pf/timesync.h"

#pragma pack(push, sp, 1)
struct ScanParameters
{
  double radial_range_min = 0.0;
  double radial_range_max = 0.0;
  bool apply_correction = true;
  int sampling_rate_max = 252000;
  int scan_time_factor = 1;

  TimeSync active_timesync;
  TimeSync passive_timesync;

  // void print()
  // {
  //   std::cout << "Scan parameters:\n"
  //             << "angular_fov: " << angular_fov << "\n"
  //             << "radial_range_min: " << radial_range_min << "\n"
  //             << "radial_range_max: " << radial_range_max << "\n"
  //             << "angle_min: " << angle_min << "\n"
  //             << "angle_max: " << angle_max << "\n"
  //             << "layers enabled: ";
  //   for(auto &layer : layers_enabled)
  //     std::cout << layer << " ";
  //   std::cout << std::endl;
  // }
};
#pragma pack(pop, sp)
