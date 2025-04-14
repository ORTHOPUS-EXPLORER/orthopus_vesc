#pragma once

#include "vescpp/vescpp/device.hpp"

namespace orthopus
{

class VESCDevice
: public vescpp::VESCDevice
{
public:
    VESCDevice(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> com);

    void processRTDataDS(vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
    void sendMeas();
private:
    std::shared_ptr<vescpp::comm::CAN> _can;
};

}