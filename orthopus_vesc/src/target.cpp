#include "orthopus_vesc/common.hpp"
#include "orthopus_vesc/target.hpp"

using namespace std::chrono_literals;

namespace orthopus
{

VESCTarget::VESCTarget(const vescpp::VESC::BoardId id, vescpp::VESCHost* host)
  : vescpp::VESCCustomHw(id, host)
  , _meas_last_tp{}
  , _meas_dt_last{0}
  , _meas_dt_min(std::numeric_limits<double>::max())
  , _meas_dt_max(0)
  , _meas_dt_avg{0}
  , _meas_dt_vvar{0}
  , _meas_dt_var{0}
  , _meas_dt_stddev{0}
  , _meas_cnt{0}
  , joint {
        false,
        false,
        0xFFFF,
        {{
            {"position"    , 0.0},
            {"velocity"    , 0.0},
            {"acceleration", 0.0},
            {"effort"      , 0.0},
        }},
        ORTHOPUS_CTRL_MODE_OFF,
        {{
            {"position" , 0.0},
            {"velocity" , 0.0},
            {"effort"   , 0.0},
        }},
    }
  , servo {
        false,
        false,
        0xFFFF, // Unused
        {{
            {"position"  , 0.5}, // FIXME: Find middle/default value for SERVO joint
        }},
        ORTHOPUS_CTRL_MODE_OFF, // Unused
        {{
            {"position"  , 0.5}, // FIXME: Find middle/default value for SERVO joint
        }},
    }
  , joints {{
        // DO NOT REORDER. There's currently an evil trick to map the right joints. 
        // You have been warned
        {
            "servo", // Will be overwritten with the name of the instanciated joint
            servo,
        },
        {
            "joint", // Will be overwritten with the name of the instanciated joint
            joint,
        },
    }}
{

}

}