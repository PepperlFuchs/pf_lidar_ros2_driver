#include "pf_driver/pf/r2000/pfsdp_2000.h"

#include "pf_driver/pf/parser_utils.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PFSDP_2000::PFSDP_2000(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<HandleInfo> info,
                       std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params)
  : PFSDPBase(node, info, config, params)
{
  node_ = node;
  declare_specific_parameters();

  parameters_handle_ =
      node_->add_on_set_parameters_callback(std::bind(&PFSDP_2000::reconfig_callback, this, std::placeholders::_1));
}

void PFSDP_2000::get_scan_parameters()
{
  auto resp = get_parameter("radial_range_min", "radial_range_max", "sampling_rate_max");
  params_->radial_range_max = parser_utils::to_float(resp["radial_range_max"]);
  params_->radial_range_min = parser_utils::to_float(resp["radial_range_min"]);
  params_->sampling_rate_max = parser_utils::to_long(resp["sampling_rate_max"]);
}

void PFSDP_2000::declare_specific_parameters()
{
}

bool PFSDP_2000::reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters)
{
  bool successful = PFSDPBase::reconfig_callback_impl(parameters);

  for (const auto& parameter : parameters)
  {
    if (parameter.get_name() == "packet_type")
    {
      std::string packet_type = parameter.as_string();
      if (packet_type == "A" || packet_type == "B" || packet_type == "C")
      {
        config_->packet_type = packet_type;
      }
      else
      {
        successful = false;
      }
    }
  }

  return successful;
}
