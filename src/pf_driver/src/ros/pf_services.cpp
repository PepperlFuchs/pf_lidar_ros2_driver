#include <memory>
#include <string>
#include <utility>

#include "pf_driver/ros/pf_services.h"

void PFServices::pfsdp_get_protocol_info(
    const std::shared_ptr<pf_interfaces::srv::PfsdpGetProtocolInfo::Request> request,
    std::shared_ptr<pf_interfaces::srv::PfsdpGetProtocolInfo::Response> response)
{
  pf_->pfsdp_info(response->protocol_name, response->version_major, response->version_minor, response->commands,
                  response->error_code, response->error_text);
}

void PFServices::pfsdp_reboot_device(const std::shared_ptr<pf_interfaces::srv::PfsdpRebootDevice::Request> request,
                                     std::shared_ptr<pf_interfaces::srv::PfsdpRebootDevice::Response> response)
{
  pf_->pfsdp_reboot(response->error_code, response->error_text);
}

void PFServices::pfsdp_factory_reset(const std::shared_ptr<pf_interfaces::srv::PfsdpFactoryReset::Request> request,
                                     std::shared_ptr<pf_interfaces::srv::PfsdpFactoryReset::Response> response)
{
  pf_->pfsdp_factory(response->error_code, response->error_text);
}

void PFServices::pfsdp_list_parameters(const std::shared_ptr<pf_interfaces::srv::PfsdpListParameters::Request> request,
                                       std::shared_ptr<pf_interfaces::srv::PfsdpListParameters::Response> response)
{
  pf_->pfsdp_list("list_parameters", "parameters", response->parameters, response->error_code, response->error_text);
}

void PFServices::pfsdp_get_parameter(const std::shared_ptr<pf_interfaces::srv::PfsdpGetParameter::Request> request,
                                     std::shared_ptr<pf_interfaces::srv::PfsdpGetParameter::Response> response)
{
  pf_->pfsdp_get("get_parameter", request->name, response->value, response->error_code, response->error_text);
}

void PFServices::pfsdp_set_parameter(const std::shared_ptr<pf_interfaces::srv::PfsdpSetParameter::Request> request,
                                     std::shared_ptr<pf_interfaces::srv::PfsdpSetParameter::Response> response)
{
  pf_->pfsdp_set("set_parameter", request->name, request->value, response->error_code, response->error_text);
}

void PFServices::pfsdp_reset_parameter(const std::shared_ptr<pf_interfaces::srv::PfsdpResetParameter::Request> request,
                                       std::shared_ptr<pf_interfaces::srv::PfsdpResetParameter::Response> response)
{
  pf_->pfsdp_reset("reset_parameter", request->name, response->error_code, response->error_text);
}

void PFServices::pfsdp_list_iq_parameters(
    const std::shared_ptr<pf_interfaces::srv::PfsdpListIqParameters::Request> request,
    std::shared_ptr<pf_interfaces::srv::PfsdpListIqParameters::Response> response)
{
  pf_->pfsdp_list("list_iq_parameters", "iq_parameters", response->iq_parameters, response->error_code,
                  response->error_text);
}

void PFServices::pfsdp_get_iq_parameter(const std::shared_ptr<pf_interfaces::srv::PfsdpGetIqParameter::Request> request,
                                        std::shared_ptr<pf_interfaces::srv::PfsdpGetIqParameter::Response> response)
{
  pf_->pfsdp_get("get_iq_parameter", request->name, response->value, response->error_code, response->error_text);
}

void PFServices::pfsdp_set_iq_parameter(const std::shared_ptr<pf_interfaces::srv::PfsdpSetIqParameter::Request> request,
                                        std::shared_ptr<pf_interfaces::srv::PfsdpSetIqParameter::Response> response)
{
  pf_->pfsdp_set("set_iq_parameter", request->name, request->value, response->error_code, response->error_text);
}
