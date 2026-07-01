#pragma once
#include "orthopus_vesc/common.hpp"
#include "orthopus_vesc/target.hpp"
#include "vescpp/vescpp/host.hpp"

namespace orthopus
{

class VESCHost : public vescpp::VESCHost
{
public:
  VESCHost(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm);
  virtual ~VESCHost();
  static std::shared_ptr<VESCHost> spawn_instance(
    vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm);
  static std::shared_ptr<VESCHost> get_instance();

  std::shared_ptr<VESCTarget> add_target(vescpp::VESC::BoardId board_id);

  void send_refs();

  bool start_streaming();

  bool set_rt_stream_rate(double rate_hz);
  bool set_aux_stream_rate(double rate_hz);

  void process_rt_data_us(
    vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8],
    const uint8_t len);
  void print_stats();

private:
  std::shared_ptr<vescpp::comm::CAN> can_;
  std::atomic_bool run_tx_th_;
  std::thread tx_th_;
  std::chrono::milliseconds rt_stream_ms_, aux_stream_ms_;
};

}  // namespace orthopus