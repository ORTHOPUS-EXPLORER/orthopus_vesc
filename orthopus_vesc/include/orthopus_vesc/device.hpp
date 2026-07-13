#pragma once

#include "vescpp/vescpp/device.hpp"

namespace orthopus
{

class VESCDevice : public vescpp::VESCDevice
{
public:
  VESCDevice(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> com);

  void process_rt_data_downstream(
    vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8],
    const uint8_t len);
  void send_measures();

private:
  std::shared_ptr<vescpp::comm::CAN> can_;
};

}  // namespace orthopus