#include <memory>
#include <string>
#include <utility>

#include "pf_driver/ros/pf_services.h"
#include "pf_driver/pf/pf_interface.h"

void pfsdp_reboot_device(PFInterface* pf,
                         const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<pf_interfaces::srv::PfsdpRebootDevice::Request> request,
                         std::shared_ptr<pf_interfaces::srv::PfsdpRebootDevice::Response> response)
{
  pf->pfsdp_reboot(response->error_code, response->error_text);
}

void pfsdp_factory_reset(PFInterface* pf,
                         const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<pf_interfaces::srv::PfsdpFactoryReset::Request> request,
                         std::shared_ptr<pf_interfaces::srv::PfsdpFactoryReset::Response> response)
{
  pf->pfsdp_factory(response->error_code, response->error_text);
}

void pfsdp_get_protocol_info(PFInterface* pf,
                             const std::shared_ptr<rmw_request_id_t> request_header,
                             const std::shared_ptr<pf_interfaces::srv::PfsdpGetProtocolInfo::Request> request,
                             std::shared_ptr<pf_interfaces::srv::PfsdpGetProtocolInfo::Response> response)
{
  pf->pfsdp_info(response->protocol_name, response->version_major, response->version_minor, response->commands,
                 response->error_code, response->error_text);
}

void pfsdp_list_parameters(PFInterface* pf,
                           const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<pf_interfaces::srv::PfsdpListParameters::Request> request,
                           std::shared_ptr<pf_interfaces::srv::PfsdpListParameters::Response> response)
{
  pf->pfsdp_list("list_parameters", "parameters", response->parameters, response->error_code, response->error_text);
}

void pfsdp_get_parameter(PFInterface* pf,
                         const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<pf_interfaces::srv::PfsdpGetParameter::Request> request,
                         std::shared_ptr<pf_interfaces::srv::PfsdpGetParameter::Response> response)
{
  pf->pfsdp_get("get_parameter", request->name, response->value, response->error_code, response->error_text);
}

void pfsdp_set_parameter(PFInterface* pf,
                         const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<pf_interfaces::srv::PfsdpSetParameter::Request> request,
                         std::shared_ptr<pf_interfaces::srv::PfsdpSetParameter::Response> response)
{
  pf->pfsdp_set("set_parameter", request->name, request->value, response->error_code, response->error_text);
}

void pfsdp_reset_parameter(PFInterface* pf,
                           const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<pf_interfaces::srv::PfsdpResetParameter::Request> request,
                           std::shared_ptr<pf_interfaces::srv::PfsdpResetParameter::Response> response)
{
  pf->pfsdp_reset("reset_parameter", request->name, response->error_code, response->error_text);
}

void pfsdp_list_iq_parameters(PFInterface* pf,
                              const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<pf_interfaces::srv::PfsdpListIqParameters::Request> request,
                              std::shared_ptr<pf_interfaces::srv::PfsdpListIqParameters::Response> response)
{
  pf->pfsdp_list("list_iq_parameters", "iq_parameters", response->iq_parameters, response->error_code,
                 response->error_text);
}

void pfsdp_get_iq_parameter(PFInterface* pf,
                            const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<pf_interfaces::srv::PfsdpGetIqParameter::Request> request,
                            std::shared_ptr<pf_interfaces::srv::PfsdpGetIqParameter::Response> response)
{
  pf->pfsdp_get("get_iq_parameter", request->name, response->value, response->error_code, response->error_text);
}

void pfsdp_set_iq_parameter(PFInterface* pf,
                            const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<pf_interfaces::srv::PfsdpSetIqParameter::Request> request,
                            std::shared_ptr<pf_interfaces::srv::PfsdpSetIqParameter::Response> response)
{
  pf->pfsdp_set("set_iq_parameter", request->name, request->value, response->error_code, response->error_text);
}

void pfsdp_reset_iq_parameter(PFInterface* pf,
                              const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<pf_interfaces::srv::PfsdpResetIqParameter::Request> request,
                              std::shared_ptr<pf_interfaces::srv::PfsdpResetIqParameter::Response> response)
{
  pf->pfsdp_reset("reset_iq_parameter", request->name, response->error_code, response->error_text);
}
