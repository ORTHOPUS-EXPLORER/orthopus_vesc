#include "orthopus_vesc/common.hpp"
#include "orthopus_vesc/host.hpp"

using namespace std::chrono_literals;

namespace orthopus
{

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
    : vescpp::VESCHost(this_id, comm.get())
    , _can(std::dynamic_pointer_cast<vescpp::comm::CAN>(comm))
    , _run_tx_th(false)
{
    if(!_can)
    {
        spdlog::error("[{}] Only CAN communication is supported right now", this_id);
        exit(0);
    }
}

VESCHost::~VESCHost()
{
    if(_run_tx_th)
    {
        _run_tx_th = false;
        _tx_th.join();
    }
}

bool VESCHost::startStreaming()
{
    if(_run_tx_th)
        return false;
    _run_tx_th = true;
    _tx_th = std::thread([this]()
    {
        auto writeRefs = [this](const std::shared_ptr<orthopus::VESCTarget>& vesc)
        {
            RTDataDS ref;
            ref.f.ctrl  = __bswap_16(vesc->ctrl_word);
            ref.f.qd    = f_u16(vesc->qd, ORTHOPUS_COMM_RT_POS_SCALE);
            ref.f.dqd   = f_u16(vesc->dqd, ORTHOPUS_COMM_RT_VEL_SCALE);
            ref.f.tauf  = f_u16(vesc->tauf, ORTHOPUS_COMM_RT_TRQ_SCALE);
            _can->write((CAN_RT_DATA_DOWNSTREAM<<8)|vesc->id, ref.raw, sizeof(RTDataDS));
        };
        while(_run_tx_th)
        {
            for(const auto& [board_id, it]:_devs)
            {
                writeRefs(std::dynamic_pointer_cast<VESCTarget>(it));
                //spdlog::error("[{}] Push Refs to 0x{:03X}", id, (CAN_RT_DATA_DOWNSTREAM<<8)|it->id);
            }
            std::this_thread::sleep_for(5ms); // 200 Hz
        }
        // Force in POS mode on the last meas
        for(auto& [_, it]: _devs)
        {
            const auto& vesc = std::dynamic_pointer_cast<VESCTarget>(it);
            vesc->ctrl_word = ORTHOPUS_CTRL_MODE_POS;
            vesc->qd = vesc->qm;
            vesc->dqd = 0.0;
            vesc->tauf = 0.0;
            writeRefs(vesc);
        }
        // Then exit
    });
    return true;
}

std::shared_ptr<VESCTarget> VESCHost::addTarget(vescpp::VESC::BoardId board_id)
{
    auto can_id = (CAN_RT_DATA_UPSTREAM<<8)|board_id;
    spdlog::debug("[{}<={}] Add CAN Handler 0x{:04X} to receive CAN_RT_DATA_UPSTREAM", id, board_id, can_id);
    _can->_can_handlers.emplace_back(can_id, std::bind(&VESCHost::processRTDataUS, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    return this->add_peer<orthopus::VESCTarget>(board_id, ::VESC::HW_TYPE_CUSTOM_MODULE);
}

void VESCHost::sendRefs()
{       
    for(auto it = _devs.begin();it != _devs.end();it++)
    {
        auto board_id = it->first;
        RTDataDS ref;
        ref.f.ctrl  = __bswap_16(0x1001);
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
    auto vesc = this->get_peer<orthopus::VESCTarget>(board_id);
    if(!vesc)
        return;
    
    //spdlog::trace("[{}] Got Upstream data from {}: {:np}", id, board_id, spdlog::to_hex(data,data+len));
    vesc->qm     =      u16_f(((uint16_t)data[1]<<8)|data[0], ORTHOPUS_COMM_RT_POS_SCALE);
    vesc->dqm    =      u16_f(((uint16_t)data[3]<<8)|data[2], ORTHOPUS_COMM_RT_VEL_SCALE);
    vesc->taum   =      u16_f(((uint16_t)data[5]<<8)|data[4], ORTHOPUS_COMM_RT_TRQ_SCALE);
    auto status  = __bswap_16(((uint16_t)data[7]<<8)|data[6]);
    spdlog::trace("[{}] Got Upstream data from {}: Pos: {:.3f}, Vel :{:.3f}, Trq: {:.3f}, Status: 0x{:04X}", id, board_id, vesc->qm, vesc->dqm, vesc->taum, status);
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
        auto vesc = this->get_peer<orthopus::VESCTarget>(board_id);
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
