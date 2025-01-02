#pragma once

#include "pf_driver/pf/pfsdp_base.h"

class PFSDP_2300 : public PFSDPBase
{
public:
  PFSDP_2300(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
             std::shared_ptr<ScanParameters> params);

  virtual void get_scan_parameters();

  void setup_param_server();

private:
  std::shared_ptr<rclcpp::Node> node_;

  virtual bool reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters) override;

  virtual void declare_specific_parameters() override;
};
