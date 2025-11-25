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
  
  , joints {{
        // DO NOT REORDER. There's currently an evil trick to map the right joints. 
        // You have been warned
        {
            "joint", // Will be overwritten with the name of the instanciated joint
            false,
            false,
            0xFFFF,
            nullptr,
            {{
                {"position"    , { false, 0.0 } },
                {"velocity"    , { false, 0.0 } },
                {"acceleration", { false, 0.0 } },
                {"effort"      , { false, 0.0 } },
            }},
            ORTHOPUS_CTRL_MODE_OFF,
            "off",
            {{
                {"mode"     , { false, 0.0 } },
                {"position" , { false, 0.0 } },
                {"velocity" , { false, 0.0 } },
                {"effort"   , { false, 0.0 } },
            }},
        }
        , {
            "servo", // Will be overwritten with the name of the instanciated joint
            false,
            false,
            0xFFFF, // Unused
            nullptr, // Unused
            {{
                {"position"  , { false, 0.5 } }, // FIXME: Find middle/default value for SERVO joint
            }},
            ORTHOPUS_CTRL_MODE_OFF, // Unused
            "off",
            {{
                {"position"  , { false, 0.5 } }, // FIXME: Find middle/default value for SERVO joint
            }},
        }
    }}
  , joint{joints[0]}
  , servo{joints[1]}
{

    pktAddHandler(::VESC::COMM_PRINT, [this](vescpp::Comm*, const vescpp::VESC::BoardId, std::shared_ptr<vescpp::VESC::Packet>& pkt)
    {
        if(auto ppkt = std::dynamic_pointer_cast<vescpp::VESC::packets::Print>(pkt); _print_hdlr && ppkt)
        {
            _print_hdlr(ppkt->str);
            return true;
        }
        return false;
    },
    true);

}

VESCTarget::joint_t* VESCTarget::getJoint(const std::string& name)
{
    for(auto& j: joints)
    {
        if(j.name == name)
            return &j;
    }
    return nullptr;
}

}