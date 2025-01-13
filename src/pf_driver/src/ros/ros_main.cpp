#include <memory>
#include <string>
#include <utility>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>

#include "pf_driver/pf/timesync.h"
#include "pf_driver/pf/pf_interface.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pf_driver");

  std::vector<std::string> pfsdp_init;
  std::string device, transport_str, scanner_ip, port, topic, frame_id, packet_type;
  int samples_per_scan, start_angle, max_num_points_scan, watchdogtimeout, skip_scans, num_layers;
  port = "0";
  num_layers = 0;
  skip_scans = 0;
  bool watchdog, apply_correction = 0;
  int timesync_interval = 0; /* ms or 0(off) */
  int timesync_period = 0;   /* ms or 0(no averaging) */
  bool timesync_regression = false; /* perform averaging(false) or linear regression(true) */

  if (!node->has_parameter("device"))
  {
    node->declare_parameter("device", device);
  }
  node->get_parameter("device", device);
  RCLCPP_INFO(node->get_logger(), "device name: %s", device.c_str());

  if (!node->has_parameter("transport"))
  {
    node->declare_parameter("transport", transport_str);
  }
  node->get_parameter("transport", transport_str);
  RCLCPP_INFO(node->get_logger(), "transport_str: %s", transport_str.c_str());

  if (!node->has_parameter("scanner_ip"))
  {
    node->declare_parameter("scanner_ip", scanner_ip);
  }
  node->get_parameter("scanner_ip", scanner_ip);
  RCLCPP_INFO(node->get_logger(), "scanner_ip: %s", scanner_ip.c_str());

  if (!node->has_parameter("port"))
  {
    node->declare_parameter("port", port);
  }
  node->get_parameter("port", port);
  RCLCPP_INFO(node->get_logger(), "port: %s", port.c_str());

  if (!node->has_parameter("start_angle"))
  {
    node->declare_parameter("start_angle", start_angle);
  }
  node->get_parameter("start_angle", start_angle);
  RCLCPP_INFO(node->get_logger(), "start_angle: %d", start_angle);

  if (!node->has_parameter("max_num_points_scan"))
  {
    node->declare_parameter("max_num_points_scan", max_num_points_scan);
  }
  node->get_parameter("max_num_points_scan", max_num_points_scan);
  RCLCPP_INFO(node->get_logger(), "max_num_points_scan: %d", max_num_points_scan);

  if (!node->has_parameter("watchdogtimeout"))
  {
    node->declare_parameter("watchdogtimeout", watchdogtimeout);
  }
  node->get_parameter("watchdogtimeout", watchdogtimeout);
  RCLCPP_INFO(node->get_logger(), "watchdogtimeout: %d", watchdogtimeout);

  if (!node->has_parameter("watchdog"))
  {
    node->declare_parameter("watchdog", watchdog);
  }
  node->get_parameter("watchdog", watchdog);
  RCLCPP_INFO(node->get_logger(), "watchdog: %d", watchdog);

  if (!node->has_parameter("timesync_interval"))
  {
    node->declare_parameter("timesync_interval", timesync_interval);
  }
  node->get_parameter("timesync_interval", timesync_interval);
  RCLCPP_INFO(node->get_logger(), "timesync_interval: %d", timesync_interval);

  if (!node->has_parameter("timesync_period"))
  {
    node->declare_parameter("timesync_period", timesync_period);
  }
  node->get_parameter("timesync_period", timesync_period);
  RCLCPP_INFO(node->get_logger(), "timesync_period: %d", timesync_period);

  if (!node->has_parameter("timesync_regression"))
  {
    node->declare_parameter("timesync_regression", timesync_regression);
  }
  node->get_parameter("timesync_regression", timesync_regression);
  RCLCPP_INFO(node->get_logger(), "timesync_regression: %s", timesync_regression ? "yes":"no");

  if (!node->has_parameter("num_layers"))
  {
    node->declare_parameter("num_layers", num_layers);
  }
  node->get_parameter("num_layers", num_layers);
  RCLCPP_INFO(node->get_logger(), "num_layers: %d", num_layers);

  if (!node->has_parameter("scan_topic"))
  {
    node->declare_parameter("scan_topic", topic);
  }
  node->get_parameter("scan_topic", topic);
  RCLCPP_INFO(node->get_logger(), "topic: %s", topic.c_str());

  if (!node->has_parameter("frame_id"))
  {
    node->declare_parameter("frame_id", frame_id);
  }
  node->get_parameter("frame_id", frame_id);
  RCLCPP_INFO(node->get_logger(), "frame_id: %s", frame_id.c_str());

  if (!node->has_parameter("packet_type"))
  {
    node->declare_parameter("packet_type", packet_type);
  }
  node->get_parameter("packet_type", packet_type);
  RCLCPP_INFO(node->get_logger(), "packet_type: %s", packet_type.c_str());

  if (!node->has_parameter("apply_correction"))
  {
    node->declare_parameter("apply_correction", apply_correction);
  }
  node->get_parameter("apply_correction", apply_correction);
  RCLCPP_INFO(node->get_logger(), "apply_correction: %d", apply_correction);

  if (!node->has_parameter("skip_scans"))
  {
    node->declare_parameter("skip_scans", skip_scans);
  }
  node->get_parameter("skip_scans", skip_scans);
  RCLCPP_INFO(node->get_logger(), "skip_scans: %d", skip_scans);

  if (!node->has_parameter("pfsdp_init"))
  {
    node->declare_parameter("pfsdp_init", pfsdp_init);
  }
  node->get_parameter("pfsdp_init", pfsdp_init);
  RCLCPP_INFO(node->get_logger(), "pfsdp_init.size: %d", (int)pfsdp_init.size());
  for (int i = 0; i < pfsdp_init.size(); ++i)
  {
    RCLCPP_INFO(node->get_logger(), "pfsdp_init[%d]: %s", i, pfsdp_init[i].c_str());
  }

  std::shared_ptr<HandleInfo> info = std::make_shared<HandleInfo>();

  info->handle_type = transport_str == "udp" ? HandleInfo::HANDLE_TYPE_UDP : HandleInfo::HANDLE_TYPE_TCP;

  info->hostname = node->get_parameter("scanner_ip").get_parameter_value().get<std::string>();
  info->port = node->get_parameter("port").get_parameter_value().get<std::string>();

  std::shared_ptr<ScanConfig> config = std::make_shared<ScanConfig>();
  config->start_angle = node->get_parameter("start_angle").get_parameter_value().get<int>();
  config->max_num_points_scan = node->get_parameter("max_num_points_scan").get_parameter_value().get<int>();
  config->skip_scans = node->get_parameter("skip_scans").get_parameter_value().get<int>();
  config->packet_type = node->get_parameter("packet_type").get_parameter_value().get<std::string>();
  config->watchdogtimeout = node->get_parameter("watchdogtimeout").get_parameter_value().get<int>();
  config->watchdog = node->get_parameter("watchdog").get_parameter_value().get<bool>();
  RCLCPP_INFO(node->get_logger(), "start_angle: %d", config->start_angle);
  config->timesync_interval = node->get_parameter("timesync_interval").get_parameter_value().get<int>();
  config->timesync_period = node->get_parameter("timesync_period").get_parameter_value().get<int>();
  config->timesync_regression = node->get_parameter("timesync_regression").get_parameter_value().get<bool>();

  std::shared_ptr<ScanParameters> params = std::make_shared<ScanParameters>();
  params->apply_correction = node->get_parameter("apply_correction").get_parameter_value().get<bool>();

  static PFInterface pf_interface(node);

  static std::shared_ptr<std::mutex> net_mtx_ = std::make_shared<std::mutex>();
  static std::shared_ptr<std::condition_variable> net_cv_ = std::make_shared<std::condition_variable>();
  static bool net_fail = false;
  static bool interrupted = false;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread t([&executor] { executor.spin(); });

  // capture SIGINT (Ctr+C) to unblock the wait on conditional variable
  std::thread t2([] {
    std::signal(SIGINT, [](int /* signum */) {
      // notify main thread about network failure
      {
        std::lock_guard<std::mutex> lock(*net_mtx_);
        interrupted = true;
      }
      net_cv_->notify_one();
      pf_interface.stop_transmission();
    });
  });

  if (rclcpp::ok())
  {
    do
    {
      net_fail = false;
      if (!pf_interface.init(info, config, params, topic, frame_id, num_layers))
      {
        RCLCPP_ERROR(node->get_logger(), "Unable to initialize device");
        if (!interrupted)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
          continue;
        }
        return -1;
      }
      if (!pf_interface.start_transmission(net_mtx_, net_cv_, net_fail))
      {
        RCLCPP_ERROR(node->get_logger(), "Unable to start scan");
        return -1;
      }
      // wait for condition variable
      {
        std::unique_lock<std::mutex> net_lock(*net_mtx_);
        net_cv_->wait(net_lock, [] { return net_fail or interrupted; });
      }

      if (net_fail)
      {
        RCLCPP_ERROR(node->get_logger(), "Network failure");
      }
      else if (interrupted)
      {
        RCLCPP_ERROR(node->get_logger(), "Interrupted");
      }
      pf_interface.terminate();
    } while (rclcpp::ok() && !interrupted);
  }

  pf_interface.stop_transmission();
  rclcpp::shutdown();
  t.join();
  t2.join();
  return 0;
}
