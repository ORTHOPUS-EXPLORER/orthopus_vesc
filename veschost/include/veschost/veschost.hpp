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

class VESCTarget
: public vescpp::VESCCustomHw
{
public:
    VESCTarget(const vescpp::VESC::BoardId id, vescpp::VESCpp* host=nullptr);
    vescpp::Time::time_point _meas_last_tp;
    double _meas_dt_last,
           _meas_dt_min,
           _meas_dt_max,
           _meas_dt_avg,
           _meas_dt_vvar,
           _meas_dt_var,
           _meas_dt_stddev;
    size_t _meas_cnt;
};

class VESCDevice
: public vescpp::VESCpp
{
public:
    VESCDevice(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> com);

    void processRTDataDS(vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
    void sendMeas();
private:
std::shared_ptr<vescpp::comm::CAN> _can;
};

class VESCHost
    : public vescpp::VESCpp
{
public:
    VESCHost(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm);
    static std::shared_ptr<VESCHost> spawnInstance(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm) ;
    static std::shared_ptr<VESCHost> getInstance();

    bool addDevice(vescpp::VESC::BoardId board_id);

    void sendRefs();

    void processRTDataUS(vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
    void printStats(void);
private:
    std::shared_ptr<vescpp::comm::CAN> _can;
};

}
