#pragma once

#include "vescpp/vescpp.hpp"
#include "vescpp/comm/can.hpp"

namespace orthopus
{
    constexpr vescpp::comm::CAN::Id CAN_RT_DATA_UPSTREAM   = 179;
    constexpr vescpp::comm::CAN::Id CAN_RT_DATA_DOWNSTREAM = 180;

    constexpr unsigned int ORTHOPUS_COMM_RT_POS_SCALE = 1000;
    constexpr unsigned int ORTHOPUS_COMM_RT_VEL_SCALE = 1000;
    constexpr unsigned int ORTHOPUS_COMM_RT_TRQ_SCALE = 1000;

    uint16_t f_u16(float v, unsigned int scale);
    float u16_f(uint16_t v, unsigned int scale);

    typedef union 
    {
        uint8_t raw[8];
        struct 
        {
            uint16_t qm;
            uint16_t dqm;
            uint16_t taum;
            uint16_t status;
        } f;
    } RTDataUS;
    typedef union 
    {
        uint8_t raw[8];
        struct
        {
            uint16_t qd;
            uint16_t dqd;
            uint16_t tauf;
            uint16_t ctrl; 
        } f;
    } RTDataDS;

    
    uint16_t f_u16(float v, unsigned int scale);
    float u16_f(uint16_t v, unsigned int scale);
}
