#include <lyra/lyra.hpp>

#include "vescpp/vescpp.hpp"
#include "vescpp/comm/can.hpp"

#include "orthopus_vesc/device.hpp"
#include "orthopus_vesc/host.hpp"
#include "orthopus_vesc/target.hpp"

int main(int argc, char**argv) 
{
    spdlog::cfg::load_env_levels();

    bool show_help = false;
    std::string can_port = "can0";
    std::vector<int> host_ids{45}, target_ids{};
    size_t targets_nb = 1;
    unsigned int host_delay = 2,
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
    
    auto can_comm = std::make_shared<vescpp::comm::CAN>(can_port);

    std::atomic_bool run_tx_th = false,
                     run_rx_th=false;
    std::unique_ptr<std::thread> tx_th,
                                 rx_th;
    std::vector<std::unique_ptr<orthopus::VESCHost>> vesc_hosts;
    std::vector<std::unique_ptr<orthopus::VESCDevice>> vesc_devs;
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
            auto& vesc = vesc_hosts.emplace_back(new orthopus::VESCHost(board_id, can_comm));
            for(const auto& id: target_ids)
                vesc->addTarget(id&0xFF);
        }
        run_tx_th = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        tx_th.reset(new std::thread([&vesc_hosts, &run_tx_th,host_delay]()
        {
            while(run_tx_th)
            {
                vescpp::Time::time_point t_st = vescpp::Time::now();
                for(auto& d: vesc_hosts)
                    d->sendRefs();
                std::this_thread::sleep_until(t_st + std::chrono::milliseconds(host_delay));
            }
            for(auto& d: vesc_hosts)
                d->printStats();
        }));
        for(auto& vh: vesc_hosts)
        {
            for(auto& [id, v]: vh->_devs)
            {
                auto& fw = v->fw();
                spdlog::info("[{0}/0x{0:02X}] FW version: {1}.{2} - HW: {3:<15s} - UUID: 0x{4:spn}", id, fw->fw_version_major, fw->fw_version_minor,  fw->hw_name.c_str(), spdlog::to_hex(fw->uuid));
            }
        }
    }
    else
    {
        spdlog::info("[{}] Start benchmark with Target IDs {}", can_port, fmt::join(target_ids, ", "));
        vesc_devs.reserve(target_ids.size());
        for(const auto& board_id: target_ids)
            vesc_devs.emplace_back(new orthopus::VESCDevice(board_id, can_comm));
        
        
        run_tx_th = true;
        tx_th.reset(new std::thread([&vesc_devs, &run_tx_th,devs_delay]()
        {
            while(run_tx_th)
            {
                vescpp::Time::time_point t_st = vescpp::Time::now();
                for(auto& d: vesc_devs)
                    d->sendMeas();
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