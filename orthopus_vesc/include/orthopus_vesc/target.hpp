#pragma once
#include <functional>
#include <string>
#include <unordered_map>

#include "vescpp/vescpp/host.hpp"
#include "vescpp/vescpp/target.hpp"

namespace orthopus
{

class VESCHost;

class VESCTarget : public vescpp::VESCCustomHw
{
public:
  typedef struct
  {
    bool in_use;
    double v;
  } intf_t;
  typedef struct joint_t
  {
    std::string name;
    bool in_use;
    bool stream;
    uint16_t status;
    std::function<void(struct joint_t&, uint16_t)> status_changed_cb;
    std::unordered_map<std::string, intf_t> meas;
    uint16_t ctrl;
    std::string ctrl_mode;
    std::unordered_map<std::string, intf_t> refs;
  } joint_t;

  VESCTarget(const vescpp::VESC::BoardId id, vescpp::VESCHost* host = nullptr);
  // TODO Set as const getter + add proper public methods to interacts
  joint_t* getJoint(const std::string& name);

public:
  // TODO private ALL thoses members
  std::function<void(const std::string&)> print_hdlr_;
  vescpp::Time::time_point _meas_last_tp;
  double _meas_dt_last, _meas_dt_min, _meas_dt_max, _meas_dt_avg, _meas_dt_vvar, _meas_dt_var,
    _meas_dt_stddev;
  size_t _meas_cnt;

  std::array<joint_t, 2> joints;
  joint_t& joint;
  joint_t& servo;
};

}  // namespace orthopus