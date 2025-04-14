#include <lyra/lyra.hpp>

#include "vescpp/vescpp.hpp"
#include "vescpp/comm/can.hpp"

namespace orthopus
{
    constexpr vescpp::comm::CAN::Id CAN_RT_DATA_UPSTREAM   = 179;
    constexpr vescpp::comm::CAN::Id CAN_RT_DATA_DOWNSTREAM = 180;

    constexpr unsigned int ORTHOPUS_COMM_RT_POS_SCALE = 1000;
    constexpr unsigned int ORTHOPUS_COMM_RT_VEL_SCALE = 1000;
    constexpr unsigned int ORTHOPUS_COMM_RT_TRQ_SCALE = 1000;

    uint16_t f_u16(float v, unsigned int scale)
    {
        return (uint16_t)(v*scale);
    }

    float u16_f(uint16_t v, unsigned int scale)
    {
        return (float)(v)/(float)scale;
    }

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
    VESCTarget(const vescpp::VESC::BoardId id)
    : vescpp::VESCCustomHw(id)
    , _meas_last_tp{}
    , _meas_dt_last{0}
    , _meas_dt_min(std::numeric_limits<double>::max())
    , _meas_dt_max(0)
    , _meas_dt_avg{0}
    , _meas_dt_vvar{0}
    , _meas_dt_var{0}
    , _meas_dt_stddev{0}
    , _meas_cnt{0}
    {}
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
    VESCDevice(vescpp::VESC::BoardId this_id, vescpp::Comm* comm)
        : vescpp::VESCpp(this_id, comm, true)
        , _can(dynamic_cast<vescpp::comm::CAN*>(_comm))
    {
        if(!_can)
        {
            spdlog::error("[{}] Only CAN communication is supported right now", this_id);
            exit(0);
        }
        _can->_can_handlers.emplace_back((CAN_RT_DATA_DOWNSTREAM<<8)|id, std::bind(&VESCDevice::processRTDataDS, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    }

    void processRTDataDS(vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
    {
    }

    void sendMeas()
    {
        RTDataUS meas;
        meas.f.status  = 0x0001;
        meas.f.qm      = f_u16(0.102, ORTHOPUS_COMM_RT_POS_SCALE);
        meas.f.dqm     = f_u16(0.304, ORTHOPUS_COMM_RT_VEL_SCALE);
        meas.f.taum    = f_u16(0.506, ORTHOPUS_COMM_RT_TRQ_SCALE);
        _can->write((CAN_RT_DATA_UPSTREAM<<8)|id, meas.raw, sizeof(RTDataUS));
    }
private:
    vescpp::comm::CAN* _can;
};

class VESCHost
    : public vescpp::VESCpp
{
public:
    VESCHost(vescpp::VESC::BoardId this_id, vescpp::Comm* comm)
        : vescpp::VESCpp(this_id, comm, false)
        , _can(dynamic_cast<vescpp::comm::CAN*>(_comm))
    {
        if(!_can)
        {
            spdlog::error("[{}] Only CAN communication is supported right now", this_id);
            exit(0);
        }
    }

    bool addDevice(vescpp::VESC::BoardId board_id)
    {
        auto can_id = (CAN_RT_DATA_UPSTREAM<<8)|board_id;
        spdlog::debug("[{}<={}] Add CAN Handler 0x{:04X} to receive CAN_RT_DATA_UPSTREAM", id, board_id, can_id);
        _can->_can_handlers.emplace_back(can_id, std::bind(&VESCHost::processRTDataUS, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        return VESCpp::add_peer<orthopus::VESCTarget>(board_id, ::VESC::HW_TYPE_CUSTOM_MODULE);
    }

    void sendRefs()
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

    void processRTDataUS(vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
    {
        const auto now = vescpp::Time::now();
        //RTDataUS meas;
        auto board_id = can_id & 0xFF;
        auto* vesc = VESCpp::get_peer<orthopus::VESCTarget>(board_id);
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

    void printStats(void)
    {
        spdlog::info("[{:>3d}] VESCHost devices statistics:", this->id);
        for(const auto& [board_id,_]: _devs)
        {
            #define STATS_FLOAT_FMT  " 7.3f"
            #define STATS_FLOAT_UNIT "ms"
            auto* vesc = VESCpp::get_peer<orthopus::VESCTarget>(board_id);
            if(!vesc)
                continue;
            spdlog::info("       - [{0}/0x{0:02X}] Received {1:10d} meas. Delta T: Last {2:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ""
                         ", Min: {3:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ", Max:{4:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ""
                         ", Avg: {5:" STATS_FLOAT_FMT "}" STATS_FLOAT_UNIT ", Var: {6:" STATS_FLOAT_FMT "}, StdDev: {7:" STATS_FLOAT_FMT "}"
            , vesc->id
            , vesc->_meas_cnt
            , vesc->_meas_dt_last
            , vesc->_meas_dt_min, vesc->_meas_dt_max
            , vesc->_meas_dt_avg, vesc->_meas_dt_var, vesc->_meas_dt_stddev);
        }
    }
private:
    vescpp::comm::CAN* _can;
};

}

int main(int argc, char**argv) 
{
    spdlog::cfg::load_env_levels();

    bool show_help = false;
    std::string can_port = "can0";
    std::vector<int> host_ids{45}, target_ids{};
    size_t targets_nb = 1;
    unsigned int host_delay = 4,
             devs_delay = 2;
    bool device_mode = false;
    auto cli = lyra::help(show_help).description("VESCHost CAN communication benchmark")
    | lyra::opt( can_port, "port")
        ["-P"]["--port"]
        ("CAN port")
    | lyra::opt( host_ids, "host_ids")
        ["-i"]["--hosts-id"]
        ("Device ID")
    | lyra::opt( host_delay, "host_delay")
        ["-D"]["--hosts-delay"]
        ("Delay between refs")
    | lyra::opt( target_ids, "target_ids")
        ["-t"]["--target-ids"]
        ("Target devices IDs")
    | lyra::opt( targets_nb, "targets_nb")
        ["-n"]["--targets-nb"]
        ("Target devices count, when providing only one target")
    | lyra::opt( device_mode)
        ["-m"]["--device-mode"]
        ("Act as device (VESC-like)")
    ;
    
    if (auto result = cli.parse( { argc, argv } ); !result)
    {
        spdlog::error("Error when parsing command line: {}. Abort", result.message());
        return 1;
    } 

    if(show_help)
    {
        std::cout << cli << std::endl; 
        return 0;
    }
    
    auto can_comm = vescpp::comm::CAN(can_port);

    std::atomic_bool run_tx_th = false,
                     run_rx_th=false;
    std::unique_ptr<std::thread> tx_th,
                                 rx_th;
    std::vector<orthopus::VESCHost> vesc_hosts;
    std::vector<orthopus::VESCDevice> vesc_devs;
    if(target_ids.size() == 1 && targets_nb > 1)
    {
        const auto t0_id = target_ids[0];
        for(size_t i=1;i<targets_nb;i++)
            target_ids.push_back(t0_id+i);
    }

    if(!device_mode)
    {
        spdlog::info("[{}] Start benchmark with Host IDs {}, Target IDs {}", can_port, fmt::join(host_ids, ", "), fmt::join(target_ids, ", "));
        for(const auto& board_id: host_ids)
        {
            auto& vesc = vesc_hosts.emplace_back(board_id, &can_comm);
            for(const auto& id: target_ids)
            {
                vesc.addDevice(id&0xFF);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        run_tx_th = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        tx_th.reset(new std::thread([&vesc_hosts, &run_tx_th,host_delay]()
        {
            while(run_tx_th)
            {
                vescpp::Time::time_point t_st = vescpp::Time::now();
                for(auto& d: vesc_hosts)
                    d.sendRefs();
                std::this_thread::sleep_until(t_st + std::chrono::milliseconds(host_delay));
            }
            for(auto& d: vesc_hosts)
                d.printStats();
        }));
        for(auto& [id, v]: vesc_hosts[0]._devs)
        {
            if(!v->fw.is_valid)
                continue;
                
            spdlog::info("[{0}/0x{0:02X}] FW version: {1}.{2} - HW: {3:<15s} - UUID: 0x{4:spn}", id, v->fw.fw_version_major, v->fw.fw_version_minor,  v->fw.hw_name.c_str(), spdlog::to_hex(v->fw.uuid));
        }
        /*const auto& can_ids = vesc.scanCAN(std::chrono::milliseconds(100));
        for(const auto& [id,typ]: can_ids)
        {
            vesc.add_peer(id,typ);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }*/
    }
    else
    {
        spdlog::info("[{}] Start benchmark with Target IDs {}", can_port, fmt::join(target_ids, ", "));
        vesc_devs.reserve(target_ids.size());
        for(const auto& board_id: target_ids)
            vesc_devs.emplace_back(board_id, &can_comm);
        
        
        run_tx_th = true;
        tx_th.reset(new std::thread([&vesc_devs, &run_tx_th,devs_delay]()
        {
            while(run_tx_th)
            {
                vescpp::Time::time_point t_st = vescpp::Time::now();
                for(auto& d: vesc_devs)
                    d.sendMeas();
                std::this_thread::sleep_until(t_st + std::chrono::milliseconds(devs_delay));
            }
        }));
    }
    spdlog::info("Press enter to exit");
    getchar();

    if(tx_th)
    {
        run_tx_th = false;
        tx_th->join();
    }
    if(rx_th)
    {
        run_rx_th = false;
        rx_th->join();
    }
    return EXIT_SUCCESS; 
}