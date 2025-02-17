#include <iostream>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>

#include "pf_driver/pf/pfsdp_base.h"
#include "pf_driver/pf/parser_utils.h"
#include "pf_driver/pf/http_helpers/http_helpers.h"

PFSDPBase::PFSDPBase(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<HandleInfo> info,
                     std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params)
  : http_interface(new HTTPInterface(info->hostname, "cmd")), node_(node), info_(info), config_(config), params_(params)
{
  parameters_handle_ =
      node_->add_on_set_parameters_callback(std::bind(&PFSDPBase::reconfig_callback, this, std::placeholders::_1));
}

void PFSDPBase::copy_status_from_json(int32_t& error_code, std::string& error_text, Json::Value json_resp)
{
  error_code = json_resp["error_code"].asInt();
  error_text = json_resp["error_text"].asString();
}

void PFSDPBase::get_parameter(const char* cmd, const std::string& name, std::string& value, int32_t& error_code,
                              std::string& error_text)
{
  Json::Value json_resp = http_interface->get(cmd, { KV("list", name) });

  copy_status_from_json(error_code, error_text, json_resp);

  if (error_code == 0)
  {
    if (json_resp[name].isArray())
    {
      value = http_helpers::from_array(json_resp[name]);
    }
    else
    {
      value = json_resp[name].asString();
    }
  }
  else
  {
    value = std::string("");
  }
}

void PFSDPBase::set_parameter(const char* cmd, const std::string& name, const std::string& value, int32_t& error_code,
                              std::string& error_text)
{
  Json::Value json_resp = http_interface->get(cmd, { KV(name, value) });

  copy_status_from_json(error_code, error_text, json_resp);
}

void PFSDPBase::reset_parameter(const char* cmd, const std::string& name, int32_t& error_code, std::string& error_text)
{
  Json::Value json_resp = http_interface->get(cmd, { KV("list", name) });

  copy_status_from_json(error_code, error_text, json_resp);
}

const std::map<std::string, std::string> PFSDPBase::get_request(const std::string& command,
                                                                const std::vector<std::string>& json_keys,
                                                                const std::initializer_list<param_type>& query)
{
  return get_request(command, json_keys, param_vector_type(query.begin(), query.end()));
}

const std::map<std::string, std::string> PFSDPBase::get_request(const std::string& command,
                                                                const std::vector<std::string>& json_keys,
                                                                const param_vector_type& query)
{
  Json::Value json_resp = http_interface->get(command, query);

  const int error_code = json_resp["error_code"].asInt();
  const std::string error_text(json_resp["error_text"].asString());

  // check if HTTP has an error
  if (error_code < 0)
  {
    std::cerr << "HTTP ERROR: " << error_text << std::endl;

    if (is_connection_failure(error_text))
    {
      if (handle_connection_failure)
      {
        handle_connection_failure();
      }
    }
    return std::map<std::string, std::string>();
  }

  // check if PFSDP has an error
  if (error_code != 0 || error_text.compare("success") != 0)

  {
    std::cout << "protocol error: " << error_code << " " << error_text << std::endl;
    return std::map<std::string, std::string>();
  }

  // Flatten values
  std::map<std::string, std::string> json_kv;
  for (std::string key : json_keys)
  {
    try
    {
      if (json_resp[key].isArray())
        json_kv[key] = http_helpers::from_array(json_resp[key]);
      else
        json_kv[key] = json_resp[key].asString();
    }
    catch (std::exception& e)
    {
      std::cout << "Invalid command or parameter (" << key << ") requested." << std::endl;
      return std::map<std::string, std::string>();
    }
  }

  return json_kv;
}

bool PFSDPBase::get_request_bool(const std::string& command, const std::vector<std::string>& json_keys,
                                 const std::initializer_list<param_type>& query)
{
  std::map<std::string, std::string> resp = get_request(command, json_keys, query);
  if (resp.empty())
  {
    return false;
  }
  return true;
}

bool PFSDPBase::is_connection_failure(const std::string& http_error)
{
  std::string error_1 = "Failed to connect to ";
  std::string error_2 = "No route to host";

  if (http_error.find(error_1) != std::string::npos && http_error.find(error_2) != std::string::npos)
  {
    return true;
  }
  return false;
}

void PFSDPBase::set_connection_failure_cb(std::function<void()> callback)
{
  handle_connection_failure = callback;
}

void PFSDPBase::list_parameters(const char* cmd, const char* out, std::vector<std::string>& params, int32_t& error_code,
                                std::string& error_text)
{
  Json::Value json_resp = http_interface->get(cmd, {});

  copy_status_from_json(error_code, error_text, json_resp);

  if (error_code == 0)
  {
    const int sz = json_resp[out].size();
    params.clear();
    for (int i = 0; i < sz; ++i)
    {
      params.push_back(json_resp[out][i].asString());
    }
  }
  else
  {
    params = std::vector<std::string>();
  }
}

void PFSDPBase::reboot(int32_t& error_code, std::string& error_text)
{
  Json::Value json_resp = http_interface->get("reboot_device", {});

  copy_status_from_json(error_code, error_text, json_resp);
}

void PFSDPBase::factory(int32_t& error_code, std::string& error_text)
{
  Json::Value json_resp = http_interface->get("factory_reset", {});

  copy_status_from_json(error_code, error_text, json_resp);
}

bool PFSDPBase::release_handle(const std::string& handle)
{
  get_request("release_handle", { "" }, { KV("handle", handle) });
  return true;
}

void PFSDPBase::info(std::string& protocol_name, int32_t& version_major, int32_t& version_minor,
                     std::vector<std::string>& commands, int32_t& error_code, std::string& error_text)
{
  Json::Value json_resp = http_interface->get("get_protocol_info", {});

  copy_status_from_json(error_code, error_text, json_resp);

  if (error_code == 0)
  {
    version_major = json_resp["version_major"].asInt();
    version_minor = json_resp["version_minor"].asInt();
    protocol_name = json_resp["protocol_name"].asString();

    const int sz = json_resp["commands"].size();
    commands.clear();
    for (int i = 0; i < sz; ++i)
    {
      commands.push_back(json_resp["commands"][i].asString());
    }
  }
}

ProtocolInfo PFSDPBase::get_protocol_info()
{
  ProtocolInfo opi;

  int32_t error_code;
  std::string error_text;

  info(opi.protocol_name, opi.version_major, opi.version_minor, opi.commands, error_code, error_text);

  return opi;
}

int64_t PFSDPBase::get_parameter_int(const std::string& param)
{
  std::map<std::string, std::string> resp = get_parameter(param);
  if (resp.empty())
  {
    return std::numeric_limits<std::int64_t>::quiet_NaN();
  }
  return parser_utils::to_long(resp[param]);
}

float PFSDPBase::get_parameter_float(const std::string& param)
{
  std::map<std::string, std::string> resp = get_parameter(param);
  if (resp.empty())
  {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return parser_utils::to_float(resp[param]);
}

std::string PFSDPBase::get_parameter_str(const std::string& param)
{
  std::map<std::string, std::string> resp = get_parameter(param);
  if (resp.empty())
  {
    return std::string("");
  }
  return resp[param];
}

void PFSDPBase::request_handle_tcp(int port)
{
  param_vector_type query;
  if (!config_->packet_type.empty())
  {
    query.push_back(KV("packet_type", config_->packet_type));
  }
  if (port != 0)
  {
    query.push_back(KV("port", port));
  }
  auto resp = get_request("request_handle_tcp", { "handle", "port" }, query);

  info_->handle = resp["handle"];
  info_->actual_port = parser_utils::to_long(resp["port"]);
}

void PFSDPBase::request_handle_udp()
{
  param_vector_type query = { KV("address", info_->endpoint), KV("port", info_->actual_port) };
  if (!config_->packet_type.empty())
  {
    query.push_back(KV("packet_type", config_->packet_type));
  }
  auto resp = get_request("request_handle_udp", { "handle", "port" }, query);
  info_->handle = resp["handle"];
}

void PFSDPBase::get_scanoutput_config(const std::string& handle)
{
  auto resp =
      get_request("get_scanoutput_config",
                  { "start_angle", "packet_type", "watchdogtimeout", "skip_scans", "watchdog", "max_num_points_scan" },
                  { KV("handle", handle) });
  config_->packet_type = resp["packet_type"];
  config_->start_angle = parser_utils::to_long(resp["start_angle"]);
  config_->watchdogtimeout = parser_utils::to_long(resp["watchdogtimeout"]);
  config_->watchdog = (resp["watchdog"] == "off") ? false : true;
  config_->skip_scans = parser_utils::to_long(resp["skip_scans"]);
  config_->max_num_points_scan = parser_utils::to_long(resp["max_num_points_scan"]);
}

bool PFSDPBase::update_scanoutput_config()
{
  param_vector_type query = { KV("handle", info_->handle) };

  query.push_back(KV("start_angle", config_->start_angle));
  if (!config_->packet_type.empty())
  {
    query.push_back(KV("packet_type", config_->packet_type));
  }
  query.push_back(KV("max_num_points_scan", config_->max_num_points_scan));
  query.push_back(KV("skip_scans", config_->skip_scans));
  if (config_->watchdogtimeout != 0)
  {
    query.push_back(KV("watchdogtimeout", config_->watchdogtimeout));
  }
  if (config_->watchdog == false)
  {
    /* Force watchdog off. Otherwise (if watchdog==true), use scanner default */
    query.push_back(KV("watchdog", "off"));
  }

  auto resp = get_request("set_scanoutput_config", { "" }, query);
  return true;
}

bool PFSDPBase::start_scanoutput()
{
  get_request("start_scanoutput", { "" }, { { "handle", info_->handle } });
  return true;
}

bool PFSDPBase::stop_scanoutput(const std::string& handle)
{
  return get_request_bool("stop_scanoutput", { "" }, { { "handle", handle } });
}

std::string PFSDPBase::get_scanoutput_config(const std::string& param, const std::string& handle)
{
  auto resp = get_request("get_scanoutput_config", { param }, { KV("handle", handle) });
  return resp[param];
}

bool PFSDPBase::feed_watchdog(const std::string& handle)
{
  return get_request_bool("feed_watchdog", { "" }, { { "handle", handle } });
}

std::string PFSDPBase::get_product()
{
  return get_parameter_str("product");
}

void PFSDPBase::get_scan_parameters()
{
  auto resp = get_parameter("radial_range_min", "radial_range_max", "sampling_rate_max");
  params_->radial_range_max = parser_utils::to_float(resp["radial_range_max"]);
  params_->radial_range_min = parser_utils::to_float(resp["radial_range_min"]);
  params_->sampling_rate_max = parser_utils::to_long(resp["sampling_rate_max"]);

  params_->layer_count = 1;
  params_->inclination_count = 1;

  Json::Value json_resp = http_interface->get("get_parameter", { KV("list", "layer_count") });
  if (json_resp["error_code"].asInt() == 0)
  {
    params_->layer_count = json_resp["layer_count"].asInt();

    json_resp = http_interface->get("get_parameter", { KV("list", "layer_inclination") });
    if (json_resp["error_code"].asInt() == 0)
    {
      /* Just check if inclinations are all zero or not. */
      Json::Value& val(json_resp["layer_inclination"]);

      if (val.isArray())
      {
        for (int i = 0; i < val.size(); ++i)
        {
          if (val[i].asDouble() != 0.0)
          {
            params_->inclination_count = params_->layer_count;
            break;
          }
        }
      }
    }
  }
}

// handle "dynamic" parameters

rcl_interfaces::msg::SetParametersResult PFSDPBase::reconfig_callback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  result.successful = reconfig_callback_impl(parameters);

  if (result.successful)
  {
    update_scanoutput_config();
  }

  return result;
}

void PFSDPBase::pfsdp_init(const rclcpp::Parameter& parameter)
{
  std::vector<std::string> pfsdp_init = parameter.as_string_array();
  for (int i = 0; i < pfsdp_init.size(); ++i)
  {
    size_t eq = pfsdp_init[i].find_first_of("=");
    if (eq != std::string::npos)
    {
      std::string name = pfsdp_init[i].substr(0, eq);
      std::string value = pfsdp_init[i].substr(eq + 1, std::string::npos);

      (void)set_parameter({ KV(name, value) });
    }
  }
}

bool PFSDPBase::reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters)
{
  bool successful = true;

  for (const auto& parameter : parameters)
  {
    if (parameter.get_name() == "pfsdp_init")
    {
      pfsdp_init(parameter);
    }
    else if (parameter.get_name() == "port")
    {
      config_->port = parameter.as_int();
    }
    else if (parameter.get_name() == "transport")
    {
      // selecting TCP as default if not UDP
      std::string transport_str = parameter.as_string();
      info_->handle_type = transport_str == "udp" ? HandleInfo::HANDLE_TYPE_UDP : HandleInfo::HANDLE_TYPE_TCP;
    }
    else if (parameter.get_name() == "watchdog")
    {
      config_->watchdog = parameter.as_bool();
    }
    else if (parameter.get_name() == "watchdogtimeout")
    {
      config_->watchdogtimeout = parameter.as_int();
    }
    else if (parameter.get_name() == "timesync_method")
    {
      std::string tsval = parameter.as_string();
      int i = TimeSync::timesync_method_name_to_int(tsval);
      if (i>=0)
      {
        config_->timesync_method = i;
      }
      else
      {
        successful = false;
      }
    }
    else if (parameter.get_name() == "timesync_averaging")
    {
      std::string tsval = parameter.as_string();
      int i = TimeSync::timesync_averaging_name_to_int(tsval);
      if (i>=0)
      {
        config_->timesync_averaging = i;
      }
      else
      {
        successful = false;
      }
    }
    else if (parameter.get_name() == "timesync_interval")
    {
      config_->timesync_interval = parameter.as_int();
    }
    else if (parameter.get_name() == "timesync_period")
    {
      config_->timesync_period = parameter.as_int();
    }
    else if (parameter.get_name() == "timesync_offset_usec")
    {
      config_->timesync_offset_usec = parameter.as_int();
    }
    else if (parameter.get_name() == "start_angle")
    {
      config_->start_angle = parameter.as_int();
    }
    else if (parameter.get_name() == "max_num_points_scan")
    {
      config_->max_num_points_scan = parameter.as_int();
    }
    else if (parameter.get_name() == "skip_scans")
    {
      config_->skip_scans = parameter.as_int();
    }
    else if (parameter.get_name() == "packet_type")
    {
      std::string packet_type = parameter.as_string();
      if (packet_type == "A" || packet_type == "B" || packet_type == "C" || packet_type == "C1")
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

void PFSDPBase::declare_common_parameters()
{
}

void PFSDPBase::setup_parameters_callback()
{
  parameters_handle_ =
      node_->add_on_set_parameters_callback(std::bind(&PFSDPBase::reconfig_callback, this, std::placeholders::_1));
}
