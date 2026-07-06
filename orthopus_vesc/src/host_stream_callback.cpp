#include "orthopus_vesc/host_stream_callback.hpp"

#include "orthopus_vesc/common.hpp"

namespace orthopus
{
// CAN writer callbacks
void realtime_write_callback(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can)
{
  RTDataDownstream ref{};
  const auto& data = vesc->joint;
  if (!(data.in_use && data.stream)) return;
  ref.f.ctrl = __bswap_16(data.ctrl);
  ref.f.qd =
    f_u16(fmodf(data.refs.at("position").v + M_PI, 2.0f * M_PI) - M_PI, ORTHOPUS_COMM_RT_POS_SCALE);
  ref.f.dqd = f_u16(data.refs.at("velocity").v, ORTHOPUS_COMM_RT_VEL_SCALE);
  ref.f.tauf = f_u16(data.refs.at("effort").v, ORTHOPUS_COMM_RT_TRQ_SCALE);
  can->write((CAN_RT_DATA_DOWNSTREAM << 8) | vesc->id, ref.raw, sizeof(RTDataDownstream));
};

void auxiliary_servo_write_callback(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can)
{
  AuxServoDataDownstream ref{};
  const auto& data = vesc->servo;
  if (!(data.in_use && data.stream)) return;
  ref.f.servo = f_u16(data.refs.at("position").v, ORTHOPUS_COMM_AUX_SERVO_SCALE);
  can->write(
    (CAN_AUX_SERVO_DATA_DOWNSTREAM << 8) | vesc->id, ref.raw, sizeof(AuxServoDataDownstream));
};

void auxiliary_config_write_callback(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can)
{
  AuxConfigDataDownstream ref{};
  const auto& data = vesc->joint;
  if (!(data.in_use && data.stream)) return;
  ref.f.impedance_control_damping =
    f_u16(data.impedance_control_damping, ORTHOPUS_COMM_IMPEDANCE_DAMPING_SCALE);
  ref.f.impedance_control_stiffness =
    f_u16(data.impedance_control_stiffness, ORTHOPUS_COMM_IMPEDANCE_STIFFNESS_SCALE);
  can->write(
    (CAN_AUX_CONFIG_DATA_DOWNSTREAM << 8) | vesc->id, ref.raw, sizeof(AuxConfigDataDownstream));
};
// Can write callbacks - end

VESCHostStreamCallback::VESCHostStreamCallback(VESCHostStreamType type, unsigned int stream_rate)
: type_(type), ms_wait_(std::chrono::milliseconds((unsigned int)(1000 / stream_rate)))
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
    callback_(vesc, can);
    refresh_next_call_time_();
  }
}

void VESCHostStreamCallback::refresh_next_call_time_()
{
  next_call_time_ = std::chrono::steady_clock::now() + ms_wait_;
}
}  // namespace orthopus