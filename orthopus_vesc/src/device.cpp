#include "orthopus_vesc/common.hpp"
#include "orthopus_vesc/device.hpp"

using namespace std::chrono_literals;

namespace orthopus
{

VESCDevice::VESCDevice(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm)
  : vescpp::VESCDevice(this_id, comm.get())
  , _can(std::dynamic_pointer_cast<vescpp::comm::CAN>(comm))
{
  if(!_can)
  {
      spdlog::error("[{}] Only CAN communication is supported right now", this_id);
      exit(0);
  }
  _can->_can_handlers.emplace_back((CAN_RT_DATA_DOWNSTREAM<<8)|id, std::bind(&VESCDevice::processRTDataDS, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

void VESCDevice::processRTDataDS(vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
}

void VESCDevice::sendMeas()
{
  RTDataUS meas;
  meas.f.status  = 0x0001;
  meas.f.qm      = f_u16(0.102, ORTHOPUS_COMM_RT_POS_SCALE);
  meas.f.dqm     = f_u16(0.304, ORTHOPUS_COMM_RT_VEL_SCALE);
  meas.f.taum    = f_u16(0.506, ORTHOPUS_COMM_RT_TRQ_SCALE);
  _can->write((CAN_RT_DATA_UPSTREAM<<8)|id, meas.raw, sizeof(RTDataUS));
}

}