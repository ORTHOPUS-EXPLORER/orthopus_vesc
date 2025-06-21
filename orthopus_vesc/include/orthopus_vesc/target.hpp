#pragma once
#include <string>
#include <unordered_map>

#include "vescpp/vescpp/host.hpp"
#include "vescpp/vescpp/target.hpp"

namespace orthopus
{

class VESCHost;

class VESCTarget
: public vescpp::VESCCustomHw
{
public:
    typedef struct
    {
        bool in_use;
        bool stream;
        uint16_t status;
        std::unordered_map<std::string, double> meas;
        uint16_t ctrl;
        std::unordered_map<std::string, double> refs;
    } joint_t;

    VESCTarget(const vescpp::VESC::BoardId id, vescpp::VESCHost* host=nullptr);
    vescpp::Time::time_point _meas_last_tp;
    double _meas_dt_last,
           _meas_dt_min,
           _meas_dt_max,
           _meas_dt_avg,
           _meas_dt_vvar,
           _meas_dt_var,
           _meas_dt_stddev;
    size_t _meas_cnt;

protected:
    friend class orthopus::VESCHost;
    joint_t joint;
    joint_t servo;
public:
    std::unordered_map<std::string, joint_t&> joints;
};

}