#pragma once

//#include <memory>
//#include <string>
//#include <utility>

#include "pf_driver/pf/pf_interface.h"
#include "pf_interfaces/srv/pfsdp_reboot_device.hpp"
#include "pf_interfaces/srv/pfsdp_factory_reset.hpp"
#include "pf_interfaces/srv/pfsdp_get_protocol_info.hpp"
#include "pf_interfaces/srv/pfsdp_get_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_set_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_reset_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_list_parameters.hpp"
#include "pf_interfaces/srv/pfsdp_get_iq_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_set_iq_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_reset_iq_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_list_iq_parameters.hpp"

void pfsdp_reboot_device(PFInterface* pf, const std::shared_ptr<pf_interfaces::srv::PfsdpRebootDevice::Request> request,
                         std::shared_ptr<pf_interfaces::srv::PfsdpRebootDevice::Response> response);

void pfsdp_factory_reset(PFInterface* pf, const std::shared_ptr<pf_interfaces::srv::PfsdpFactoryReset::Request> request,
                         std::shared_ptr<pf_interfaces::srv::PfsdpFactoryReset::Response> response);

void pfsdp_get_protocol_info(PFInterface* pf,
                             const std::shared_ptr<pf_interfaces::srv::PfsdpGetProtocolInfo::Request> request,
                             std::shared_ptr<pf_interfaces::srv::PfsdpGetProtocolInfo::Response> response);

void pfsdp_list_parameters(PFInterface* pf,
                           const std::shared_ptr<pf_interfaces::srv::PfsdpListParameters::Request> request,
                           std::shared_ptr<pf_interfaces::srv::PfsdpListParameters::Response> response);

void pfsdp_get_parameter(PFInterface* pf, const std::shared_ptr<pf_interfaces::srv::PfsdpGetParameter::Request> request,
                         std::shared_ptr<pf_interfaces::srv::PfsdpGetParameter::Response> response);

void pfsdp_set_parameter(PFInterface* pf, const std::shared_ptr<pf_interfaces::srv::PfsdpSetParameter::Request> request,
                         std::shared_ptr<pf_interfaces::srv::PfsdpSetParameter::Response> response);

void pfsdp_reset_parameter(PFInterface* pf,
                           const std::shared_ptr<pf_interfaces::srv::PfsdpResetParameter::Request> request,
                           std::shared_ptr<pf_interfaces::srv::PfsdpResetParameter::Response> response);

void pfsdp_list_iq_parameters(PFInterface* pf,
                              const std::shared_ptr<pf_interfaces::srv::PfsdpListIqParameters::Request> request,
                              std::shared_ptr<pf_interfaces::srv::PfsdpListIqParameters::Response> response);

void pfsdp_get_iq_parameter(PFInterface* pf,
                            const std::shared_ptr<pf_interfaces::srv::PfsdpGetIqParameter::Request> request,
                            std::shared_ptr<pf_interfaces::srv::PfsdpGetIqParameter::Response> response);

void pfsdp_set_iq_parameter(PFInterface* pf,
                            const std::shared_ptr<pf_interfaces::srv::PfsdpSetIqParameter::Request> request,
                            std::shared_ptr<pf_interfaces::srv::PfsdpSetIqParameter::Response> response);

void pfsdp_reset_iq_parameter(PFInterface* pf,
                              const std::shared_ptr<pf_interfaces::srv::PfsdpResetIqParameter::Request> request,
                              std::shared_ptr<pf_interfaces::srv::PfsdpResetIqParameter::Response> response);
