#include "veschost/veschost.hpp"

namespace orthopus
{

uint16_t f_u16(float v, unsigned int scale)
{
    return (uint16_t)(v*scale);
}

float u16_f(uint16_t v, unsigned int scale)
{
    return (float)(v)/(float)scale;
}

VESCTarget::VESCTarget(const vescpp::VESC::BoardId id, vescpp::VESCpp* host)
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

VESCDevice::VESCDevice(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm)
    : vescpp::VESCpp(this_id, comm.get(), true)
    , _can(std::dynamic_pointer_cast<vescpp::comm::CAN>(comm))
{
    if(!_can)
    {
        spdlog::error("[{}] Only CAN communication is supported right now", this_id);
        exit(0);
    }
    _can->_can_handlers.emplace_back((CAN_RT_DATA_DOWNSTREAM<<8)|id, std::bind(&VESCDevice::processRTDataDS, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

void VESCDevice::processRTDataDS(vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
}

void VESCDevice::sendMeas()
{
    RTDataUS meas;
    meas.f.status  = 0x0001;
    meas.f.qm      = f_u16(0.102, ORTHOPUS_COMM_RT_POS_SCALE);
    meas.f.dqm     = f_u16(0.304, ORTHOPUS_COMM_RT_VEL_SCALE);
    meas.f.taum    = f_u16(0.506, ORTHOPUS_COMM_RT_TRQ_SCALE);
    _can->write((CAN_RT_DATA_UPSTREAM<<8)|id, meas.raw, sizeof(RTDataUS));
}

static std::shared_ptr<VESCHost> vesc_instance{nullptr};

std::shared_ptr<VESCHost> VESCHost::spawnInstance(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm) 
{
    vesc_instance.reset(new VESCHost(this_id, comm));
    return vesc_instance;
}

std::shared_ptr<VESCHost> VESCHost::getInstance() 
{
    return vesc_instance;
}

VESCHost::VESCHost(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm)
    : vescpp::VESCpp(this_id, comm.get(), false)
    , _can(std::dynamic_pointer_cast<vescpp::comm::CAN>(comm))
{
    if(!_can)
    {
        spdlog::error("[{}] Only CAN communication is supported right now", this_id);
        exit(0);
    }
}

bool VESCHost::addDevice(vescpp::VESC::BoardId board_id)
{
    auto can_id = (CAN_RT_DATA_UPSTREAM<<8)|board_id;
    spdlog::debug("[{}<={}] Add CAN Handler 0x{:04X} to receive CAN_RT_DATA_UPSTREAM", id, board_id, can_id);
    _can->_can_handlers.emplace_back(can_id, std::bind(&VESCHost::processRTDataUS, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    return VESCpp::add_peer<orthopus::VESCTarget>(board_id, ::VESC::HW_TYPE_CUSTOM_MODULE) != nullptr;
}

void VESCHost::sendRefs()
{       
    for(auto it = _devs.begin();it != _devs.end();it++)
    {
        auto board_id = it->first;
        RTDataDS ref;
        ref.f.ctrl  = 0x1001;
        ref.f.qd    = f_u16(1.102, ORTHOPUS_COMM_RT_POS_SCALE);
        ref.f.dqd   = f_u16(1.304, ORTHOPUS_COMM_RT_VEL_SCALE);
        ref.f.tauf  = f_u16(1.506, ORTHOPUS_COMM_RT_TRQ_SCALE);
        _can->write((CAN_RT_DATA_DOWNSTREAM<<8)|board_id, ref.raw, sizeof(RTDataDS));
    }
}

void VESCHost::processRTDataUS(vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
    const auto now = vescpp::Time::now();
    //RTDataUS meas;
    auto board_id = can_id & 0xFF;
    auto vesc = VESCpp::get_peer<orthopus::VESCTarget>(board_id);
    if(!vesc)
        return;
    
    //spdlog::trace("[{}] Got Upstream data from {}: {:np}", id, board_id, spdlog::to_hex(data,data+len));
    auto qm     = u16_f(((uint16_t)data[0]<<8)|data[1], ORTHOPUS_COMM_RT_TRQ_SCALE);
    auto dqm    = u16_f(((uint16_t)data[2]<<8)|data[3], ORTHOPUS_COMM_RT_VEL_SCALE);
    auto taum   = u16_f(((uint16_t)data[4]<<8)|data[5], ORTHOPUS_COMM_RT_POS_SCALE);
    auto status = ((uint16_t)data[6]<<8)|data[7];
    spdlog::trace("[{}] Got Upstream data from {}: Pos: {:.3f}, Vel :{:.3f}, Trq: {:.3f}, Status: 0x{:04X}", id, board_id, qm, dqm, taum, status);
    vesc->_meas_cnt++;
    if(vesc->_meas_last_tp.time_since_epoch().count() > 0)
    {
        const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - vesc->_meas_last_tp).count()/1000.0; //ms
        if(dt < vesc->_meas_dt_min)
            vesc->_meas_dt_min = dt;
        if(dt > vesc->_meas_dt_max)
            vesc->_meas_dt_max = dt;
        vesc->_meas_dt_last = dt;
        auto avg_p = vesc->_meas_dt_avg;
        vesc->_meas_dt_avg += (dt - vesc->_meas_dt_avg)/vesc->_meas_cnt;
        if(vesc->_meas_cnt > 1)
        {
            vesc->_meas_dt_vvar += (dt - avg_p)*(dt - vesc->_meas_dt_avg);
            vesc->_meas_dt_var = sqrt(vesc->_meas_dt_vvar)/(vesc->_meas_cnt-1);
            vesc->_meas_dt_stddev = sqrt(vesc->_meas_dt_var);
        }
    }
    vesc->_meas_last_tp = now;
}

void VESCHost::printStats(void)
{
    spdlog::info("[{:>3d}] VESCHost devices statistics:", this->id);
    for(const auto& [board_id,_]: _devs)
    {
        #define STATS_FLOAT_FMT  " 8.5f"
        #define STATS_FLOAT_UNIT "ms"
        auto vesc = VESCpp::get_peer<orthopus::VESCTarget>(board_id);
        if(!vesc)
            continue;
        spdlog::info("  - [{0}/0x{0:02X}] Received {1:10d} meas. Delta T: Last {2:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ""
                        ", Min: {3:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ", Max:{4:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ""
                        ", Avg: {5:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ", Var: {6:" STATS_FLOAT_FMT "}, StdDev: {7:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT
        , vesc->id
        , vesc->_meas_cnt
        , vesc->_meas_dt_last
        , vesc->_meas_dt_min, vesc->_meas_dt_max
        , vesc->_meas_dt_avg, vesc->_meas_dt_var, vesc->_meas_dt_stddev);
    }
}


}
