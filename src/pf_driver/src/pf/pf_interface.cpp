#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <rcutils/logging.h>
#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/ros/pf_services.h"
#include "pf_driver/ros/laser_scan_publisher.h"
#include "pf_driver/ros/point_cloud_publisher.h"
#include "pf_driver/communication/udp_transport.h"
#include "pf_driver/communication/tcp_transport.h"

PFInterface::PFInterface(std::shared_ptr<rclcpp::Node> node) : node_(node), state_(PFState::UNINIT)
{
}

bool PFInterface::init(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
                       std::shared_ptr<ScanParameters> params, const std::string& topic, const std::string& frame_id)
{
  config_ = config;
  info_ = info;
  params_ = params;

  params_->active_timesync.init(config_->timesync_period, config_->timesync_offset_usec,
                                (config_->timesync_averaging == TIMESYNC_AVERAGING_REGRESSION));
  params_->passive_timesync.init(config_->timesync_period, config_->timesync_offset_usec,
                                 (config_->timesync_averaging == TIMESYNC_AVERAGING_REGRESSION));

  topic_ = topic;
  frame_id_ = frame_id;

  has_iq_parameters_ = false;

  protocol_interface_ = std::make_shared<PFSDPBase>(node_, info_, config_, params_);

  // This is the first time ROS communicates with the device
  auto opi = protocol_interface_->get_protocol_info();
  if (opi.isError)
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to communicate with device. Please check the IP address");
    return false;
  }

  if (opi.protocol_name != "pfsdp")
  {
    RCLCPP_ERROR(node_->get_logger(), "Incorrect protocol");
    return false;
  }

  if ((std::find(opi.commands.begin(), opi.commands.end(), "request_handle_udp") == opi.commands.end()) &&
      (std::find(opi.commands.begin(), opi.commands.end(), "request_handle_tcp") == opi.commands.end()))
  {
    RCLCPP_ERROR(node_->get_logger(), "The device doesn't support scan data output");
    return false;
  }

  // update global config_
  protocol_interface_->get_scan_parameters();

  if (params_->layer_count > 1 && params_->inclination_count > 1)
  {
    params_->scan_time_factor = params_->layer_count;
    reader_ = std::shared_ptr<PFPacketReader>(
        new PointcloudPublisher(node_, config_, params_, topic.c_str(), frame_id.c_str(), params_->layer_count));
  }
  else
  {
    params_->scan_time_factor = 1;
    reader_ = std::shared_ptr<PFPacketReader>(
        new LaserscanPublisher(node_, config_, params_, topic.c_str(), frame_id.c_str()));
  }

  if (std::find(opi.commands.begin(), opi.commands.end(), "list_iq_parameters") != opi.commands.end())
  {
    has_iq_parameters_ = true;
  }

  product_ = protocol_interface_->get_product();
  RCLCPP_INFO(node_->get_logger(), "Device found: %s", product_.c_str());

  // release previous handles
  if (!prev_handle_.empty())
  {
    protocol_interface_->release_handle(prev_handle_);
  }

  // apply pfsdp_init name=value setup
  rclcpp::Parameter pfsdp_init_param;
  if (node_->get_parameter("pfsdp_init", pfsdp_init_param))
  {
    protocol_interface_->pfsdp_init(pfsdp_init_param);
  }

  // add services
  add_pf_services();

  if (info->handle_type == HandleInfo::HANDLE_TYPE_UDP)
  {
    transport_ = std::make_unique<UDPTransport>(info->hostname, config->port);
    if (!transport_->connect())
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to establish UDP connection");
      return false;
    }

    info->endpoint = transport_->get_host_ip();
    info->actual_port = transport_->get_port();
    protocol_interface_->request_handle_udp();
  }
  else if (info->handle_type == HandleInfo::HANDLE_TYPE_TCP)
  {
    transport_ = std::make_unique<TCPTransport>(info->hostname);
    protocol_interface_->request_handle_tcp(config->port);
    // if initially port was not set, request_handle sets it
    // set the updated port in transport
    transport_->set_port(info->actual_port);
    if (!transport_->connect())
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to establish TCP connection");
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Incorrect transport option");
    return false;
  }

  if (info->handle.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not acquire communication handle");
    return false;
  }

  prev_handle_ = info_->handle;

  /* Configure sensor from ScanConfig (only explicitly set options) */
  protocol_interface_->update_scanoutput_config();

  /* Update our ScanConfig from sensor (all options) */
  protocol_interface_->get_scanoutput_config(info_->handle);

  protocol_interface_->setup_parameters_callback();
  protocol_interface_->set_connection_failure_cb(std::bind(&PFInterface::connection_failure_cb, this));
  change_state(PFState::INIT);
  return true;
}

void PFInterface::change_state(PFState state)
{
  if (state_ == state)
    return;
  state_ = state;  // Can use this function later
                   // to check state transitions
  std::string text;
  if (state_ == PFState::UNINIT)
    text = "Uninitialized";
  if (state_ == PFState::INIT)
    text = "Initialized";
  if (state_ == PFState::RUNNING)
    text = "Running";
  if (state_ == PFState::SHUTDOWN)
    text = "Shutdown";
  if (state_ == PFState::ERROR)
    text = "Error";
  RCLCPP_INFO(node_->get_logger(), "Device state changed to %s", text.c_str());
}

bool PFInterface::can_change_state(PFState state)
{
  return true;
}

bool PFInterface::start_transmission(std::shared_ptr<std::mutex> net_mtx,
                                     std::shared_ptr<std::condition_variable> net_cv, bool& net_fail)
{
  if (state_ != PFState::INIT)
    return false;

  if (pipeline_ && pipeline_->is_running())
    return true;

  pipeline_ = get_pipeline(config_->packet_type, net_mtx, net_cv, net_fail);
  if (!pipeline_ || !pipeline_->start())
    return false;

  protocol_interface_->start_scanoutput();
  if (config_->watchdog)
  {
    double timeout_s = config_->watchdogtimeout / 1000.0;
    if (timeout_s < 0.1)
    {
      timeout_s = 0.1;
    }
    start_watchdog_timer(timeout_s);
  }
  if (config_->timesync_method == TIMESYNC_METHOD_REQUESTS)
  {
    start_timesync_timer(config_->timesync_interval);
  }
  change_state(PFState::RUNNING);
  return true;
}

// What happens to the connection_ obj?
void PFInterface::stop_transmission()
{
  if (state_ != PFState::RUNNING)
    return;
  protocol_interface_->stop_scanoutput(info_->handle);
  protocol_interface_->release_handle(info_->handle);
  change_state(PFState::INIT);
}

void PFInterface::terminate()
{
  if (!pipeline_)
    return;

  if (watchdog_timer_)
  {
    watchdog_timer_->cancel();
    watchdog_timer_ = rclcpp::TimerBase::SharedPtr();
  }
  if (timesync_timer_)
  {
    timesync_timer_->cancel();
    timesync_timer_ = rclcpp::TimerBase::SharedPtr();
  }

  pipeline_->terminate();
  pipeline_.reset();
  protocol_interface_.reset();
  transport_.reset();
  change_state(PFState::UNINIT);
}

bool PFInterface::init()
{
  return init(info_, config_, params_, topic_, frame_id_);
}

void PFInterface::start_watchdog_timer(float duration)
{
  // dividing the watchdogtimeout by 2 to have a “safe” feed time within the defined timeout
  float feed_time_sec = std::min(duration, 60.0f) / 2.0f;
  int feed_time = feed_time_sec * 1000;
  watchdog_timer_ =
      node_->create_wall_timer(std::chrono::milliseconds(feed_time), std::bind(&PFInterface::feed_watchdog, this));
}

void PFInterface::feed_watchdog()
{
  protocol_interface_->feed_watchdog(info_->handle);
}

void PFInterface::start_timesync_timer(unsigned interval)
{
  timesync_timer_ =
      node_->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&PFInterface::update_timesync, this));
}

void PFInterface::update_timesync(void)
{
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  rclcpp::Time when(rclcpp::Clock().now());
  rclcpp::Time start(steady_clock.now());

  auto resp = protocol_interface_->get_parameter("system_time_raw");

  rclcpp::Time end(steady_clock.now());

  const uint64_t sensor_time = stoull(resp["system_time_raw"]);
  params_->active_timesync.update(sensor_time, (end - start).nanoseconds() / 1000, when);
}

void PFInterface::on_shutdown()
{
  RCLCPP_INFO(node_->get_logger(), "Shutting down pipeline!");
  // stop_transmission();
}

void PFInterface::connection_failure_cb()
{
  std::cout << "handling connection failure" << std::endl;
  terminate();
  std::cout << "terminated" << std::endl;
  while (!init())
  {
    std::cout << "trying to reconnect..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void PFInterface::add_pf_services(void)
{
  // Add PFSDP services as appropriate for the given product features
  std::string basename = std::string(node_->get_namespace()) + std::string(node_->get_name()) + std::string("/pfsdp_");
  std::string svcname;

  std::shared_ptr<PFServices> pf_services = std::make_shared<PFServices>(this);

  svcname = basename + "get_protocol_info";
  info_service_ = node_->create_service<pf_interfaces::srv::PfsdpGetProtocolInfo>(
      svcname.c_str(),
      std::bind(&PFServices::pfsdp_get_protocol_info, pf_services, std::placeholders::_1, std::placeholders::_2));

  svcname = basename + "reboot_device";
  reboot_service_ = node_->create_service<pf_interfaces::srv::PfsdpRebootDevice>(
      svcname.c_str(),
      std::bind(&PFServices::pfsdp_reboot_device, pf_services, std::placeholders::_1, std::placeholders::_2));

  svcname = basename + "factory_reset";
  factory_service_ = node_->create_service<pf_interfaces::srv::PfsdpFactoryReset>(
      svcname.c_str(),
      std::bind(&PFServices::pfsdp_factory_reset, pf_services, std::placeholders::_1, std::placeholders::_2));

  svcname = basename + "list_parameters";
  listparams_service_ = node_->create_service<pf_interfaces::srv::PfsdpListParameters>(
      svcname.c_str(),
      std::bind(&PFServices::pfsdp_list_parameters, pf_services, std::placeholders::_1, std::placeholders::_2));

  svcname = basename + "get_parameter";
  getparam_service_ = node_->create_service<pf_interfaces::srv::PfsdpGetParameter>(
      svcname.c_str(),
      std::bind(&PFServices::pfsdp_get_parameter, pf_services, std::placeholders::_1, std::placeholders::_2));

  svcname = basename + "set_parameter";
  setparam_service_ = node_->create_service<pf_interfaces::srv::PfsdpSetParameter>(
      svcname.c_str(),
      std::bind(&PFServices::pfsdp_set_parameter, pf_services, std::placeholders::_1, std::placeholders::_2));

  svcname = basename + "reset_parameter";
  resetparam_service_ = node_->create_service<pf_interfaces::srv::PfsdpResetParameter>(
      svcname.c_str(),
      std::bind(&PFServices::pfsdp_reset_parameter, pf_services, std::placeholders::_1, std::placeholders::_2));

  if (has_iq_parameters_)
  {
    svcname = basename + "list_iq_parameters";
    listiqparams_service_ = node_->create_service<pf_interfaces::srv::PfsdpListIqParameters>(
        svcname.c_str(),
        std::bind(&PFServices::pfsdp_list_iq_parameters, pf_services, std::placeholders::_1, std::placeholders::_2));

    svcname = basename + "get_iq_parameter";
    getiqparam_service_ = node_->create_service<pf_interfaces::srv::PfsdpGetIqParameter>(
        svcname.c_str(),
        std::bind(&PFServices::pfsdp_get_iq_parameter, pf_services, std::placeholders::_1, std::placeholders::_2));

    svcname = basename + "set_iq_parameter";
    setiqparam_service_ = node_->create_service<pf_interfaces::srv::PfsdpSetIqParameter>(
        svcname.c_str(),
        std::bind(&PFServices::pfsdp_set_iq_parameter, pf_services, std::placeholders::_1, std::placeholders::_2));
  }
}

std::unique_ptr<Pipeline> PFInterface::get_pipeline(const std::string& packet_type, std::shared_ptr<std::mutex> net_mtx,
                                                    std::shared_ptr<std::condition_variable> net_cv, bool& net_fail)
{
  std::shared_ptr<Parser<PFPacket>> parser;
  std::shared_ptr<Writer<PFPacket>> writer;
  RCLCPP_INFO(node_->get_logger(), "PacketType is: %s", packet_type.c_str());
  if (packet_type == "A")
  {
    parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_A_Parser);
  }
  else if (packet_type == "B")
  {
    parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_B_Parser);
  }
  else if (packet_type == "C")
  {
    parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_C_Parser);
  }
  else if (packet_type == "C1")
  {
    parser = std::unique_ptr<Parser<PFPacket>>(new PFR2300_C1_Parser);
  }
  if (!parser)
  {
    return nullptr;
  }
  writer =
      std::shared_ptr<Writer<PFPacket>>(new PFWriter<PFPacket>(std::move(transport_), parser, node_->get_logger()));
  return std::make_unique<Pipeline>(writer, reader_, std::bind(&PFInterface::connection_failure_cb, this), net_mtx,
                                    net_cv, net_fail);
}

void PFInterface::pfsdp_reboot(int32_t& error_code, std::string& error_text)
{
  protocol_interface_->reboot(error_code, error_text);
}

void PFInterface::pfsdp_factory(int32_t& error_code, std::string& error_text)
{
  protocol_interface_->factory(error_code, error_text);
}

void PFInterface::pfsdp_info(std::string& protocol_name, int32_t& version_major, int32_t& version_minor,
                             std::vector<std::string>& commands, int32_t& error_code, std::string& error_text)
{
  protocol_interface_->info(protocol_name, version_major, version_minor, commands, error_code, error_text);
}

void PFInterface::pfsdp_list(const char* cmd, const char* out, std::vector<std::string>& params, int32_t& error_code,
                             std::string& error_text)
{
  protocol_interface_->list_parameters(cmd, out, params, error_code, error_text);
}

void PFInterface::pfsdp_get(const char* cmd, const std::string& name, std::string& value, int32_t& error_code,
                            std::string& error_text)
{
  protocol_interface_->get_parameter(cmd, name, value, error_code, error_text);
}

void PFInterface::pfsdp_set(const char* cmd, const std::string& name, const std::string& value, int32_t& error_code,
                            std::string& error_text)
{
  protocol_interface_->set_parameter(cmd, name, value, error_code, error_text);
}

void PFInterface::pfsdp_reset(const char* cmd, const std::string& name, int32_t& error_code, std::string& error_text)
{
  protocol_interface_->reset_parameter(cmd, name, error_code, error_text);
}
