#pragma once
#include "vescpp/vescpp/host.hpp"

#include "orthopus_vesc/common.hpp"
#include "orthopus_vesc/target.hpp"

namespace orthopus
{

class VESCHost
    : public vescpp::VESCHost
{
public:
    VESCHost(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm);
    ~VESCHost();
    static std::shared_ptr<VESCHost> spawnInstance(vescpp::VESC::BoardId this_id, std::shared_ptr<vescpp::Comm> comm) ;
    static std::shared_ptr<VESCHost> getInstance();

    std::shared_ptr<VESCTarget> addTarget(vescpp::VESC::BoardId board_id);

    void sendRefs();

    bool startStreaming();

    bool setRTStreamRate(double rate_hz);
    bool setAuxStreamRate(double rate_hz);

    void processRTDataUS(vescpp::comm::CAN* can, const vescpp::comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
    void printStats(void);
private:
    std::shared_ptr<vescpp::comm::CAN> _can;
    std::atomic_bool _run_tx_th;
    std::thread _tx_th;
    std::chrono::milliseconds _rt_stream_ms,
                              _aux_stream_ms;
};

}