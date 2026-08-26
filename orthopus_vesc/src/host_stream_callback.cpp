#include "orthopus_vesc/host_stream_callback.hpp"

#include "orthopus_vesc/common.hpp"

namespace orthopus
{
// CAN writer callbacks
void realtime_write_callback(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can,
  bool simulate_response)
{
  RTDataDownstream ref{};
  const auto& data = vesc->get_joint();
  if (!(data.in_use && data.stream)) return;

  // Watchdog: if VESCInterface::write() hasn't refreshed last_cmd_timepoint recently (ros2_control
  // update loop stalled/crashed), forget the last real command and force a stationary state instead.
  const bool stale = data.last_command_timepoint.has_value() &&
                     (vescpp::Time::now() - data.last_command_timepoint.value().load()) >
                       data.command_watchdog_timeout;

  uint16_t ctrl = data.ctrl;
  double pos_ref = data.refs.at("position").v;
  double vel_ref = data.refs.at("velocity").v;
  double trq_ref = data.refs.at("effort").v;
  if (stale)
  {
    ctrl = (ctrl & ~orthopus::ORTHOPUS_CTRL_MODE_MSK) | orthopus::ORTHOPUS_CTRL_MODE_OFF;
    vel_ref = 0.0;
    trq_ref = 0.0;
    pos_ref = data.meas.at("position").v;
    spdlog::warn(
      "[{}] Command watchdog timeout: no write() in over {}ms, forcing safe state", vesc->id,
      data.command_watchdog_timeout.count());
  }

  ref.f.ctrl = __bswap_16(ctrl);
  ref.f.qd = f_u16(fmodf(pos_ref + M_PI, 2.0f * M_PI) - M_PI, ORTHOPUS_COMM_RT_POS_SCALE);
  ref.f.dqd = f_u16(vel_ref, ORTHOPUS_COMM_RT_VEL_SCALE);
  ref.f.tauf = f_u16(trq_ref, ORTHOPUS_COMM_RT_TRQ_SCALE);
  can->write((CAN_RT_DATA_DOWNSTREAM << 8) | vesc->id, ref.raw, sizeof(RTDataDownstream));
  if (simulate_response)
  {
    auto& data = vesc->acquire_joint();
    data.meas.at("position").v = data.refs.at("position").v;
    data.meas.at("velocity").v = data.refs.at("velocity").v;
    data.meas.at("effort").v = data.refs.at("effort").v;
  }
};

void auxiliary_servo_write_callback(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can,
  bool simulate_response)
{
  AuxServoDataDownstream ref{};
  const auto& data = vesc->get_servo();
  if (!(data.in_use && data.stream)) return;
  ref.f.servo = f_u16(data.refs.at("position").v, ORTHOPUS_COMM_AUX_SERVO_SCALE);
  can->write(
    (CAN_AUX_SERVO_DATA_DOWNSTREAM << 8) | vesc->id, ref.raw, sizeof(AuxServoDataDownstream));
  if (simulate_response)
  {
    auto& data = vesc->acquire_servo();
    data.meas.at("position").v = data.refs.at("position").v;
  }
};

void auxiliary_config_write_callback(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can,
  [[maybe_unused]] bool simulate_response)
{
  AuxConfigDataDownstream ref{};
  const auto& data = vesc->get_joint();
  if (!(data.in_use && data.stream && data.impedance_control_damping.has_value() &&
        data.impedance_control_stiffness.has_value()))
    return;
  ref.f.impedance_control_damping =
    f_u16(data.impedance_control_damping.value(), ORTHOPUS_COMM_IMPEDANCE_DAMPING_SCALE);
  ref.f.impedance_control_stiffness =
    f_u16(data.impedance_control_stiffness.value(), ORTHOPUS_COMM_IMPEDANCE_STIFFNESS_SCALE);
  can->write(
    (CAN_AUX_CONFIG_DATA_DOWNSTREAM << 8) | vesc->id, ref.raw, sizeof(AuxConfigDataDownstream));
};
// Can write callbacks - end

VESCHostStreamCallback::VESCHostStreamCallback(
  VESCHostStreamType type, unsigned int stream_rate, bool simulate_response)
: type_(type),
  ms_wait_(std::chrono::milliseconds((unsigned int)(1000 / stream_rate))),
  simulate_response_(simulate_response)
{
  switch (type)
  {
    case orthopus::VESCHostStreamType::REALTIME:
      callback_ = realtime_write_callback;
      break;
    case orthopus::VESCHostStreamType::AUXILIARY_GRIPPER:
      callback_ = auxiliary_servo_write_callback;
      break;
    case orthopus::VESCHostStreamType::AUXILIARY_CONFIG:
      callback_ = auxiliary_config_write_callback;
      break;
  }
}

[[nodiscard]] VESCHostStreamType VESCHostStreamCallback::get_type() const { return type_; }
[[nodiscard]] std::chrono::steady_clock::time_point VESCHostStreamCallback::get_next_call_time()
  const
{
  return next_call_time_;
}

void VESCHostStreamCallback::execute(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can)
{
  if (callback_ && vesc && can)
  {
    callback_(vesc, can, simulate_response_);
    refresh_next_call_time_();
  }
}

void VESCHostStreamCallback::refresh_next_call_time_()
{
  next_call_time_ = std::chrono::steady_clock::now() + ms_wait_;
}
}  // namespace orthopus