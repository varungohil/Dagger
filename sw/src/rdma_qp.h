/**
 * @file rpc_threaded_server.h
 * @brief Implementation of the RPC server.
 * @author Nikita Lazarev
 */
#ifndef _RDMA_V2_H_
#define _RDMA_V2_H_

#include <memory>
#include <mutex>
#include <vector>

#include "nic.h"
#include "rpc_server_thread.h"
#include "queuepair_v2.h"

namespace dagger {

/// This class implements a wrapper on top of the RpcServerThread class
/// encapsulating server threads, and abstracts away low-level details of
/// the communication with the nic.
class RDMA {
 public:
  RDMA() = default;
  std::vector<std::shared_ptr<QueuePairV2>> qp_pool_;

  /// Create the RPC server object with the given number of threads and based on
  /// the nic with the hardware MMIO address @param base_nic_addr.
  RDMA(uint64_t base_nic_addr, size_t max_num_of_threads,
       size_t max_qp_pool_size);
  ~RDMA();

  /// A wrapper on top of the nic's init/start/stop API.
  //int init_nic(int bus);
  int init_nic_slave(int bus);
  int init_nic(int bus);
  int start_nic();
  int stop_nic();

  /// A wrapper on top of the nic's hardware error checker API.
  int check_hw_errors() const;

  /// Run a new listening thread with the RPC handler @param rpc_callback and
  /// pin its dispatch thread to the CPU @param pim_cpu.
//   int run_new_listening_thread(const RpcServerCallBack_Base* rpc_callback,
//                                int pin_cpu = -1);

  /// Stop all currently running RPC threads.
//   int stop_all_listening_threads();

  // Connection management API.
//   int connect(const IPv4& client_addr, ConnectionId c_id,
//               ConnectionFlowId c_flow_id);
//   int disconnect(ConnectionId c_id);

  /// Run the perf_thread with the corresponsing @param perf_mask as the perf
  /// event filter and the post-processing callback function @param callback.
  /// The perf_thread runs periodically, reads hardware performance counters and
  /// calls the callback function to perform all sort of processing on the
  /// performance data.
  int run_perf_thread(Nic::NicPerfMask perf_mask,
                      void (*callback)(const std::vector<uint64_t>&));

  /// Set the desired load balancing scheme which will be used to distribute
  /// requests across the RpcServerThread's.
  // void set_lb(int lb);

  int make_qp();
  int send(uint16_t queue_pair_num);
  int recv(uint16_t queue_pair_num);
  int stop_recv(uint16_t queue_pair_num);
  int add_send_queue_entry(uint16_t queue_pair_num, volatile int* data_addr, size_t data_size);
  int add_recv_queue_entry(uint16_t queue_pair_num, volatile int* data_addr, size_t data_size);
  bool is_data_available(uint16_t queue_pair_num);
  int connect_qp(uint16_t queue_pair_num, ConnectionId c_id, const IPv4& server_addr, uint16_t remote_qp_num, uint16_t p_key, uint32_t q_key);

 private:
  size_t max_num_of_threads_;
  size_t max_qp_pool_size_;
  uint64_t base_nic_addr_;

  /// The NIC is shared by all threads in the pool and owned by the
  /// RDMA class.
  std::unique_ptr<Nic> nic_;

  /// Thread pool.
  // std::vector<std::unique_ptr<RpcServerThread>> threads_;
  size_t thread_cnt_;

  /// QueuePair pool.
  // std::vector<std::unique_ptr<QueuePairV2>> qp_pool_;
  size_t qp_pool_cnt_;

  /// Sync.
  std::mutex mtx_;

  /// Status of the underlying hardware nic.
  bool nic_is_started_;
};

}  // namespace dagger

#endif
