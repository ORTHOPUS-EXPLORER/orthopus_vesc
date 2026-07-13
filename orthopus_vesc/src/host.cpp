#include "orthopus_vesc/host.hpp"

#include <spdlog/spdlog.h>

#include <memory>
#include <optional>

#include "orthopus_vesc/common.hpp"
#include "orthopus_vesc/host_stream_callback.hpp"
using namespace std::chrono_literals;

#ifdef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef DEG2RAD_f
#define DEG2RAD_f(deg) ((deg) * (float)(M_PI / 180.0))
#endif
#ifndef RPM2RADS_f
#define RPM2RADS_f(rpm) ((rpm) * (float)(M_PI / 30.0))
#endif
#ifndef RAD2DEG_f
#define RAD2DEG_f(rad) ((rad) * (float)(180.0 / M_PI))
#endif
#ifndef RADS2RPM_f
#define RADS2RPM_f(rads) ((rads) * (float)(30.0 / M_PI))
#endif
namespace orthopus
{

static std::shared_ptr<VESCHost> vesc_instance{nullptr};

std::shared_ptr<VESCHost> VESCHost::spawn_instance(
  vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm,
  unsigned int realtime_stream_rate, unsigned int auxiliary_gripper_stream_rate,
  unsigned int auxiliary_config_stream_rate)
{
  vesc_instance = std::make_shared<VESCHost>(
    this_id, comm, realtime_stream_rate, auxiliary_gripper_stream_rate,
    auxiliary_config_stream_rate);
  return vesc_instance;
}

std::shared_ptr<VESCHost> VESCHost::get_instance() { return vesc_instance; }

VESCHost::VESCHost(
  vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm,
  unsigned int realtime_stream_rate, unsigned int auxiliary_gripper_stream_rate,
  unsigned int auxiliary_config_stream_rate)
: vescpp::VESCHost(this_id, comm.get()),
  can_(std::dynamic_pointer_cast<vescpp::comm::CAN>(comm)),
  run_tx_th_(true)
{
  if (!can_)
  {
    spdlog::error("[{}] Only CAN communication is supported right now", this_id);
    exit(0);
  }

  if (
    realtime_stream_rate == 0 || auxiliary_gripper_stream_rate == 0 ||
    auxiliary_config_stream_rate == 0)
  {
    spdlog::error(
      "[{}] A stream rate was set to 0 Hz, please check all the stream rates provided", this_id);
    exit(0);
  }

  stream_list_ = {
    VESCHostStreamCallback(VESCHostStreamType::REALTIME, realtime_stream_rate),
    VESCHostStreamCallback(VESCHostStreamType::AUXILIARY_GRIPPER, auxiliary_gripper_stream_rate),
    VESCHostStreamCallback(VESCHostStreamType::AUXILIARY_CONFIG, auxiliary_config_stream_rate)};

  tx_th_ = std::thread(
    [this]()
    {
      while (run_tx_th_)
      {
        auto now = std::chrono::steady_clock::now();
        std::optional<std::chrono::steady_clock::time_point> next_callback_time;

        for (auto& stream_callback : stream_list_)
        {
          auto current_next_call_time = stream_callback.get_next_call_time();
          if (current_next_call_time <= now)
          {
            for (const auto& [board_id, it] : _devs)
            {
              stream_callback.execute(std::dynamic_pointer_cast<VESCTarget>(it), can_);
            }
          }
          // Get updated next call time
          current_next_call_time = stream_callback.get_next_call_time();
          // Look for the smaller next callback time to wait
          if (
            !next_callback_time.has_value() || current_next_call_time < next_callback_time.value())
          {
            next_callback_time = current_next_call_time;
          }
        }
        if (next_callback_time.has_value())
        {
          std::this_thread::sleep_until(next_callback_time.value());
        }
      }
      // Force in POS mode on the last meas
      // Note: I suspect it's never called because of vesc already freed before getting there
      for (auto& [_, it] : _devs)
      {
        auto vesc = std::dynamic_pointer_cast<VESCTarget>(it);
        auto& data = vesc->acquire_joint();
        if (!(data.in_use && data.stream)) return;
        data.ctrl = ORTHOPUS_CTRL_MODE_POS;
        data.refs.at("position").v = data.meas.at("position").v;
        data.refs.at("velocity").v = 0.0;
        data.refs.at("effort").v = 0.0;
        orthopus::realtime_write_callback(vesc, can_);
      }
    });
}

VESCHost::~VESCHost()
{
  if (run_tx_th_)
  {
    run_tx_th_ = false;
    tx_th_.join();
  }
}

std::shared_ptr<VESCTarget> VESCHost::add_target(
  vescpp::VESC::BoardId board_id, bool check_firmware_version)
{
  auto can_id = (CAN_RT_DATA_UPSTREAM << 8) | board_id;
  spdlog::debug(
    "[{}<={}] Add CAN Handler 0x{:04X} to receive CAN_RT_DATA_UPSTREAM (check firmware ? [{}])", id,
    board_id, can_id, check_firmware_version);
  can_->_can_handlers.emplace_back(
    can_id, [this](
              vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t* data,
              uint8_t len) { process_rt_data_us(can, can_id, data, len); });
  return this->add_peer<orthopus::VESCTarget>(
    board_id, ::VESC::HW_TYPE_CUSTOM_MODULE, check_firmware_version);
}

void VESCHost::send_refs()
{
  for (auto& it : _devs)
  {
    auto board_id = it.first;
    RTDataDownstream ref{};
    ref.f.ctrl = __bswap_16(0x1001);
    ref.f.qd = f_u16(1.102, ORTHOPUS_COMM_RT_POS_SCALE);
    ref.f.dqd = f_u16(1.304, ORTHOPUS_COMM_RT_VEL_SCALE);
    ref.f.tauf = f_u16(1.506, ORTHOPUS_COMM_RT_TRQ_SCALE);
    can_->write((CAN_RT_DATA_DOWNSTREAM << 8) | board_id, ref.raw, sizeof(RTDataDownstream));
  }
}

void VESCHost::process_rt_data_us(
  vescpp::comm::CAN* can, vescpp::comm::CAN::Id can_id, const uint8_t data[8], uint8_t len)
{
  const auto now = vescpp::Time::now();
  //RTDataUpstream meas;
  auto board_id = can_id & 0xFF;
  auto vesc = this->get_peer<orthopus::VESCTarget>(board_id);
  if (!vesc) return;

  auto& jdata = vesc->acquire_joint();  // Only handle joint for now, ignore servo
  if (!jdata.in_use) return;

  //spdlog::trace("[{}] Got Upstream data from {}: {:np}", id, board_id, spdlog::to_hex(data,data+len));
  float raw_position = u16_f(((uint16_t)data[1] << 8) | data[0], ORTHOPUS_COMM_RT_POS_SCALE);

  // Wrap to [-π, π] range for ROS convention
  jdata.meas.at("position") = {true, fmodf(raw_position + M_PI, 2.0f * M_PI) - M_PI};
  jdata.meas.at("velocity") = {
    true, u16_f(((uint16_t)data[3] << 8) | data[2], ORTHOPUS_COMM_RT_VEL_SCALE)};
  jdata.meas.at("effort") = {
    true, u16_f(((uint16_t)data[5] << 8) | data[4], ORTHOPUS_COMM_RT_TRQ_SCALE)};
  auto status = __bswap_16(((uint16_t)data[7] << 8) | data[6]);
  if (jdata.status != status)

  {
    auto old_st = jdata.status;
    jdata.status = status;
    if (jdata.status_changed_cb) jdata.status_changed_cb(jdata, old_st);
    //spdlog::warn("[{}] Status word changed from 0x{:4x} to 0x{:4x}", vesc->id, data.status, status);
  }

  //spdlog::trace("[{}] Got Upstream data from {}: Pos: {:.3f}, Vel :{:.3f}, Trq: {:.3f}, Status: 0x{:04X}", id, board_id, vesc->qm, vesc->dqm, vesc->taum, status);
  vesc->_meas_cnt++;
  if (vesc->_meas_last_tp.time_since_epoch().count() > 0)
  {
    const auto dt =
      std::chrono::duration_cast<std::chrono::microseconds>(now - vesc->_meas_last_tp).count() /
      1000.0;  //ms
    if (dt < vesc->_meas_dt_min) vesc->_meas_dt_min = dt;
    if (dt > vesc->_meas_dt_max) vesc->_meas_dt_max = dt;
    vesc->_meas_dt_last = dt;
    auto avg_p = vesc->_meas_dt_avg;
    vesc->_meas_dt_avg += (dt - vesc->_meas_dt_avg) / vesc->_meas_cnt;
    if (vesc->_meas_cnt > 1)
    {
      vesc->_meas_dt_vvar += (dt - avg_p) * (dt - vesc->_meas_dt_avg);
      vesc->_meas_dt_var = sqrt(vesc->_meas_dt_vvar) / (vesc->_meas_cnt - 1);
      vesc->_meas_dt_stddev = sqrt(vesc->_meas_dt_var);
    }
  }
  vesc->_meas_last_tp = now;
}

void VESCHost::print_stats()
{
  spdlog::info("[{:>3d}] VESCHost devices statistics:", this->id);
  for (const auto& [board_id, _] : _devs)
  {
#define STATS_FLOAT_FMT " 8.5f"
#define STATS_FLOAT_UNIT "ms"
    auto vesc = this->get_peer<orthopus::VESCTarget>(board_id);
    if (!vesc) continue;
    spdlog::info(
      "  - [{0}/0x{0:02X}] Received {1:10d} meas. Delta T: Last {2:" STATS_FLOAT_FMT
      "}" STATS_FLOAT_UNIT
      ""
      ", Min: {3:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ", Max:{4:" STATS_FLOAT_FMT
      "}" STATS_FLOAT_UNIT
      ""
      ", Avg: {5:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ", Var: {6:" STATS_FLOAT_FMT
      "}, StdDev: {7:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT,
      vesc->id, vesc->_meas_cnt, vesc->_meas_dt_last, vesc->_meas_dt_min, vesc->_meas_dt_max,
      vesc->_meas_dt_avg, vesc->_meas_dt_var, vesc->_meas_dt_stddev);
  }
}

}  // namespace orthopus
