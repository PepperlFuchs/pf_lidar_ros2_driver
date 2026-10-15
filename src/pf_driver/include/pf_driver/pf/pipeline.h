#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <thread>
#include <vector>
#include <memory>
#include <mutex>
#include "pf_driver/queue/readerwriterqueue.h"
#include "pf_driver/pf/writer.h"
#include "pf_driver/pf/reader.h"
#include "pf_driver/pf/drop_counters.h"

class PFPacket;

class Pipeline
{
public:
  Pipeline(std::shared_ptr<Writer<PFPacket>> writer, std::shared_ptr<Reader<PFPacket>> reader,
           std::function<void()> func, std::shared_ptr<std::mutex> net_mtx,
           std::shared_ptr<std::condition_variable> net_cv, bool& net_fail,
           std::shared_ptr<DropCounters> counters = nullptr);

  bool start();

  void terminate();

  bool is_running();

  void on_shutdown();

  std::size_t queue_capacity() const;

private:
  moodycamel::BlockingReaderWriterQueue<std::unique_ptr<PFPacket>> queue_;  // the queue basically stored scan data
  std::shared_ptr<Reader<PFPacket>> reader_;
  std::shared_ptr<Writer<PFPacket>> writer_;
  std::function<void()> shutdown;
  std::atomic<bool> running_, shutdown_;
  std::thread reader_thread_, writer_thread_;
  std::mutex mutex_;
  std::shared_ptr<std::mutex> net_mtx_;
  std::shared_ptr<std::condition_variable> net_cv_;
  bool& net_fail_;
  std::shared_ptr<DropCounters> counters_;

  void run_writer();

  void run_reader();
};
