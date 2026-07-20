#pragma once

#include <chrono>
#include <memory>

#include "orthopus_vesc/common.hpp"
#include "orthopus_vesc/target.hpp"

namespace orthopus
{
enum class VESCHostStreamType
{
  REALTIME,
  AUXILIARY_GRIPPER,
  AUXILIARY_CONFIG
};

void realtime_write_callback(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can,
  bool simulate_response);
void auxiliary_servo_write_callback(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can,
  bool simulate_response);
void auxiliary_config_write_callback(
  const std::shared_ptr<orthopus::VESCTarget>& vesc, const std::shared_ptr<vescpp::comm::CAN>& can,
  bool simulate_response);

class VESCHostStreamCallback
{
public:
  VESCHostStreamCallback(VESCHostStreamType type, unsigned int stream_rate, bool simulate_response);
  [[nodiscard]] VESCHostStreamType get_type() const;
  [[nodiscard]] std::chrono::steady_clock::time_point get_next_call_time() const;
  void execute(
    const std::shared_ptr<orthopus::VESCTarget>& vesc,
    const std::shared_ptr<vescpp::comm::CAN>& can);

private:
  void refresh_next_call_time_();

  VESCHostStreamType type_;
  std::chrono::milliseconds ms_wait_;
  bool simulate_response_;
  std::function<void(
    const std::shared_ptr<orthopus::VESCTarget>& vesc,
    const std::shared_ptr<vescpp::comm::CAN>& can, bool simulate_response)>
    callback_;
  std::chrono::steady_clock::time_point next_call_time_;
};
}  // namespace orthopus