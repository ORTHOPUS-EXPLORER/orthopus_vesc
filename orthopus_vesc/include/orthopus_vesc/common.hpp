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


    constexpr uint16_t ORTHOPUS_CTRL_MODE_OFF = 0x0000;
    constexpr uint16_t ORTHOPUS_CTRL_MODE_POS = 0x0001;
    constexpr uint16_t ORTHOPUS_CTRL_MODE_VEL = 0x0002;
    constexpr uint16_t ORTHOPUS_CTRL_MODE_TRQ = 0x0004;
    constexpr uint16_t ORTHOPUS_CTRL_MODE_MSK = 0x000F;
    constexpr uint16_t ORTHOPUS_CTRL_MODE_ERR = 0x0010;

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
