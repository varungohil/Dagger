#ifndef _RPC_THREADED_SERVER_H_
#define _RPC_THREADED_SERVER_H_

#include <vector>
#include <memory>
#include <mutex>

#include "nic.h"
#include "rpc_server_thread.h"

namespace frpc {

class RpcThreadedServer {
public:
    RpcThreadedServer() = default;
    RpcThreadedServer(uint64_t base_nic_addr, size_t max_num_of_threads);
    ~RpcThreadedServer();

    int init_nic(int bus);
    int start_nic();
    int stop_nic();

    int check_hw_errors() const;

    int run_new_listening_thread(const RpcServerCallBack_Base* rpc_callback, int pin_cpu = -1);

    int stop_all_listening_threads();

    // Open connection
    int connect(const IPv4& client_addr,
                ConnectionId c_id,
                ConnectionFlowId c_flow_id);
    int disconnect(ConnectionId c_id);

    // Run perf thread on the nic
    int run_perf_thread(NicPerfMask perf_mask,
                        void(*callback)(const std::vector<uint64_t>&));

    void set_lb(int lb);

private:
    size_t max_num_of_threads_;
    uint64_t base_nic_addr_;

    // The NIC is shared by all threads in the pool
    // and owned by the RpcThreadedServer class
    std::unique_ptr<Nic> nic_;

    // Thread pool
    std::vector<std::unique_ptr<RpcServerThread>> threads_;

    // Thread counter
    size_t thread_cnt_;

    // Sync
    std::mutex mtx_;

    // Status
    bool nic_is_started_;

};

}  // namespace frpc

#endif