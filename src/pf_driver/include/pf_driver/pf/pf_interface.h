#pragma once

#include <string>
#include <memory>
#include <future>

#include <rclcpp/rclcpp.hpp>

#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/pf/pf_writer.h"
#include "pf_driver/pf/pf_packet_reader.h"
#include "pf_driver/pf/pipeline.h"
#include "pf_driver/communication/transport.h"
#include "pf_driver/pf/r2000/pfsdp_2000.h"
#include "pf_driver/pf/r2300/pfsdp_2300.h"

#include "pf_interfaces/srv/pfsdp_get_protocol_info.hpp"
#include "pf_interfaces/srv/pfsdp_reboot_device.hpp"
#include "pf_interfaces/srv/pfsdp_factory_reset.hpp"
#include "pf_interfaces/srv/pfsdp_list_parameters.hpp"
#include "pf_interfaces/srv/pfsdp_get_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_set_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_reset_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_list_iq_parameters.hpp"
#include "pf_interfaces/srv/pfsdp_get_iq_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_set_iq_parameter.hpp"
#include "pf_interfaces/srv/pfsdp_reset_iq_parameter.hpp"

class PFInterface
{
public:
  PFInterface(std::shared_ptr<rclcpp::Node> node);

  bool init(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
            std::shared_ptr<ScanParameters> params, const std::string& topic, const std::string& frame_id,
            const uint16_t num_layers);

  bool start_transmission(std::shared_ptr<std::mutex> net_mtx, std::shared_ptr<std::condition_variable> net_cv,
                          bool& net_fail);
  void stop_transmission();
  void terminate();

  void pfsdp_reboot(int32_t& error_code, std::string& error_text);
  void pfsdp_factory(int32_t& error_code, std::string& error_text);
  void pfsdp_info(std::string& protocol_name, int32_t& version_major, int32_t& version_minor,
        std::vector<std::string>& commands, int32_t& error_code, std::string& error_text);
  void pfsdp_list(const char* cmd, const char* out, std::vector<std::string>& params, int32_t& error_code, std::string& error_text);
  void pfsdp_get(const char* cmd, const std::string& name, std::string& value, int32_t& error_code, std::string& error_text);
  void pfsdp_set(const char* cmd, const std::string& name, const std::string& value, int32_t& error_code, std::string& error_text);
  void pfsdp_reset(const char* cmd, const std::string& name, int32_t& error_code, std::string& error_text);

  std::vector<std::string> pfsdp_list_iq(void);
  std::string pfsdp_get_iq(const std::string& name);
  void pfsdp_set_iq(const std::string& name, const std::string& value);

private:
  using PipelinePtr = std::unique_ptr<Pipeline>;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<Transport> transport_;
  std::shared_ptr<PFSDPBase> protocol_interface_;
  PipelinePtr pipeline_;
  std::shared_ptr<Reader<PFPacket>> reader_;

  rclcpp::Service<pf_interfaces::srv::PfsdpGetProtocolInfo>::SharedPtr info_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpRebootDevice>::SharedPtr reboot_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpFactoryReset>::SharedPtr factory_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpListParameters>::SharedPtr listparams_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpGetParameter>::SharedPtr getparam_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpSetParameter>::SharedPtr setparam_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpResetParameter>::SharedPtr resetparam_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpListIqParameters>::SharedPtr listiqparams_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpGetIqParameter>::SharedPtr getiqparam_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpSetIqParameter>::SharedPtr setiqparam_service_;
  rclcpp::Service<pf_interfaces::srv::PfsdpResetIqParameter>::SharedPtr resetiqparam_service_;

  std::string topic_;
  std::string frame_id_;
  uint16_t num_layers_;
  std::string product_;

  std::shared_ptr<HandleInfo> info_;
  std::shared_ptr<ScanConfig> config_;
  std::shared_ptr<ScanParameters> params_;
  std::string prev_handle_;

  enum class PFState
  {
    UNINIT,
    INIT,
    RUNNING,
    SHUTDOWN,
    ERROR
  };
  PFState state_;

  bool has_iq_parameters_;

  bool init();

  void change_state(PFState state);
  bool can_change_state(PFState state);

  void start_watchdog_timer(float duration);
  void feed_watchdog();  // timer based
  void on_shutdown();

  // factory functions
  bool handle_version(int major_version, int minor_version, int device_family, const std::string& topic,
                      const std::string& frame_id, const uint16_t num_layers);
  void add_pf_services(void);
  PipelinePtr get_pipeline(const std::string& packet_type, std::shared_ptr<std::mutex> net_mtx,
                           std::shared_ptr<std::condition_variable> net_cv, bool& net_fail);
  void connection_failure_cb();
};
