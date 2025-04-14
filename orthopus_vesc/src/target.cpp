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
{

}

}