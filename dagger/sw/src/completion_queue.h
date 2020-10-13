#ifndef _COMPLETION_QUEUE_H
#define _COMPLETION_QUEUE_H

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <utility>

#include "rpc_header.h"
#include "rx_queue.h"

namespace frpc {

/// Completion queue for non-blocking RPCs
/// Currently requires a separate management thread
///
class CompletionQueue {
public:
    CompletionQueue();
    CompletionQueue(size_t rpc_client_id, volatile char* rx_buff, size_t mtu_size_bytes);
    ~CompletionQueue();

    void bind();
    void unbind();

    size_t get_number_of_completed_requests() const;

    RpcPckt pop_response();

#ifdef PROFILE_LATENCY
    void init_latency_profile(uint64_t* timestamp_recv);
#endif

private:
    void _PullListen();

private:
    size_t rpc_client_id_;

    // Rx queue
    RxQueue rx_queue_;

    // Thread
    std::thread thread_;
    std::atomic<bool> stop_signal_;

    // CQ
    std::vector<RpcPckt> cq_;

    // Sync
    std::mutex cq_lock_;

#ifdef PROFILE_LATENCY
    // Latency profiler
    uint64_t* lat_prof_timestamp;
#endif

};

}  // namespace frpc

#endif
