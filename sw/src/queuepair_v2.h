/**
 * @file rpc_server_thread.h
 * @brief Implementation of the RPC server thread.
 * @author Nikita Lazarev
 */
#ifndef _QueuePairV2_H_
#define _QueuePairV2_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

#include "connection_manager.h"
#include "nic.h"
#include "rpc_call.h"
#include "rpc_header.h"
#include "rx_queue.h"
#include "tx_queue.h"

namespace dagger {

struct QueueElem {
  volatile int* data_addr; //Do we need volatile here?
  size_t data_size;
};

class QueuePairV2 {
 public:
  /// Construct server thread based on the nic's @param nic flow id
  /// @param nic_flow_id. The @param callback specifies the server RPC stubs
  /// generated by the RPC codegenerator and inheriting the
  /// RpcServerCallBack_Base class.
  QueuePairV2(const Nic* nic, size_t nic_flow_id, uint16_t queue_pair_num);
  virtual ~QueuePairV2();

  /// Connection management API.
  int connect(ConnectionId c_id, const IPv4& server_addr, uint16_t remote_qp_num, uint16_t p_key, uint32_t q_key);
  int disconnect(ConnectionId c_id);

  /// These functions start/stop polling in the dispatch thread.
  /// @param pin_cpu is used to pin the dispatch thread to the given CPU core.
  int start_listening();
  void stop_listening();

  /// Get associated bound completion queue.
  // CompletionQueue* get_completion_queue() const;

  int send();
  int recv();
  int stop_recv();

  void add_send_queue_entry(volatile int* data_addr, size_t data_size);
  void add_recv_queue_entry(volatile int* data_addr, size_t data_size);

  // void append_elem(std::queue<QueueElem> q);

  bool is_data_available();

  uint16_t get_qp_num();
  void operator_(const RpcPckt* rpc_in, TxQueue& tx_queue);
  std::queue<QueueElem> recv_q;
  std::queue<QueueElem> send_q;

 private:
  // Dispatch thread
  void _PullListen();

 private:
  uint16_t queue_pair_num_;
  uint16_t remote_qp_num_;
  uint16_t p_key_;
  uint32_t q_key_;
  bool is_data_available_ = 0; 



  // std::vector<std::unique_ptr<QueueElem>> recv_q;
  // std::vector<std::unique_ptr<QueueElem>> send_q;

  // Underlying nic.
  const Nic* nic_;
  size_t nic_flow_id_;

  // Underlying Tx and RX queues.
  TxQueue tx_queue_;
  RxQueue rx_queue_;

  // Underlying completion queue
  // std::unique_ptr<CompletionQueue> cq_;

  ConnectionId c_id_;

  // Threads and signals.
  std::thread thread_;
  std::atomic<bool> stop_signal_;


};

}  // namespace dagger

#endif  // _QueuePairV2_H_