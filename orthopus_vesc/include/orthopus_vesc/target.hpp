#pragma once
#include <string>
#include <unordered_map>

#include "vescpp/vescpp/host.hpp"
#include "vescpp/vescpp/target.hpp"

namespace orthopus
{

class VESCTarget
: public vescpp::VESCCustomHw
{
public:
    typedef struct
    {
        uint16_t status;
        std::unordered_map<std::string, double&> meas;
        uint16_t ctrl;
        std::unordered_map<std::string, double&> refs;
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
    double qm, dqm, ddqm, taum,
           qd, dqd, tauf;
    double sqm, sqd;
    std::unordered_map<std::string, joint_t> joints = 
    {{
        {
            "servo",
            {
                0x0000,
                {{
                    {"position"  , sqm},
                }},
                0x0000,
                {{
                    {"position"  , sqd},
                }},
            },
        },
        {
            "joint",
            {
                0x0000,
                {{
                    {"position"    , qm  },
                    {"velocity"    , dqm },
                    {"acceleration", ddqm},
                    {"effort"      , taum},
                }},
                0x0000,
                {{
                    {"position" , qd},
                    {"velocity" , dqd},
                    {"effort"   , tauf},
                }},
            },
        },
    }};
};

}