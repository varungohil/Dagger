#include "rdma_qp.h"

#include "logger.h"
#include "nic_ccip_dma.h"
#include "nic_ccip_mmio.h"
#include "nic_ccip_polling.h"

namespace dagger {

/// We always apply rx/tx LLC anti-alizasing for the server when using huge
/// pages.
static constexpr size_t rx_tx_anti_aliasing =
    dagger::cfg::sys::enable_hugepages ? 1 : 0;

RDMA::RDMA(uint64_t base_nic_addr, size_t max_num_of_threads,
           size_t max_qp_pool_size)
    : max_num_of_threads_(max_num_of_threads),
      max_qp_pool_size_(max_qp_pool_size),
      thread_cnt_(0),
      qp_pool_cnt_(0),
      base_nic_addr_(base_nic_addr),
      nic_is_started_(false) {}

RDMA::~RDMA() {
  // Stop all threads.
  // if (threads_.size() > 0) {
  //   stop_all_listening_threads();
  // }

  // Stop nic.
  if (nic_is_started_) {
    stop_nic();
  }
}

/// This function initializes the backend's nic depending on the exact type of
/// the nic. The function performs four actions to initialize the nic.
int RDMA::init_nic(int bus) {
  // (1) Create nic.
  // In contrast to rpc_client_pool, the server's nic is always the master
  // (even in the ASE mode).
#ifdef NIC_CCIP_POLLING
#  pragma message "compiling Nic to run in polling mode"
  // Simple case so far: number of NIC flows = max_num_of_threads_.
  nic_ = std::unique_ptr<Nic>(
      new NicPollingCCIP(base_nic_addr_, max_num_of_threads_, true));
#elif NIC_CCIP_MMIO
// MMIO intefrace only works either with write-combine buffering or AVX
// intrinsics.
#  pragma message "compiling Nic to run in MMIO mode"
  // Simple case so far: number of NIC flows = max_num_of_threads_.
  nic_ = std::unique_ptr<Nic>(
      new NicMmioCCIP(base_nic_addr_, max_num_of_threads_, true));
#elif NIC_CCIP_DMA
#  pragma message "compiling Nic to run in DMA mode"
  // Simple case so far: number of NIC flows = max_num_of_threads_.
  nic_ = std::unique_ptr<Nic>(
      new NicDmaCCIP(base_nic_addr_, max_num_of_threads_, true));
#else
#  error Nic CCI-P mode is not specified
#endif

  // (2) Connect to nic.
  int res = nic_->connect_to_nic(bus);
  if (res != 0) return res;
  FRPC_INFO("Connected to NIC on the bus %x\n", bus);

  // (3) Configure the nic dataplane. Of course, all the clients in this pool
  // share the same configuration.
  PhyAddr cl_phy_addr = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0xFF};
  IPv4 cl_ipv4_addr("192.168.0.2", 0);

  res = nic_->configure_data_plane(rx_tx_anti_aliasing);
  if (res != 0) return res;

  // (4) Run hardware initialization.
  res = nic_->initialize_nic(cl_phy_addr, cl_ipv4_addr);
  if (res != 0) return res;

  return 0;
}

int RDMA::start_nic() {
  int res = nic_->start();
  if (res != 0) {
    FRPC_ERROR("Failed to start NIC\n");
    return res;
  }

  nic_is_started_ = true;
  FRPC_INFO("NIC is started\n");
  return 0;
}

int RDMA::stop_nic() {
  int res = nic_->stop();
  if (res != 0) {
    FRPC_ERROR("Failed to stop NIC\n");
    return res;
  }

  nic_is_started_ = false;
  FRPC_INFO("Server NIC is stopped\n");
  return 0;
}

int RDMA::check_hw_errors() const { return nic_->check_hw_errors(); }

// int RDMA::run_new_listening_thread(const RpcServerCallBack_Base* rpc_callback,
//                                    int pin_cpu) {
//   std::unique_lock<std::mutex> lck(mtx_);

//   if (thread_cnt_ < max_num_of_threads_) {
//     threads_.push_back(std::unique_ptr<RpcServerThread>(new RpcServerThread(
//         nic_.get(), thread_cnt_, thread_cnt_, rpc_callback)));

//     int r = threads_.back().get()->start_listening(pin_cpu);
//     if (r != 0) {
//       threads_.pop_back();
//       return 1;
//     }

//     ++thread_cnt_;
//     return 0;
//   } else {
//     FRPC_ERROR("Max number of rpc threads is reached: %zu\n",
//                max_num_of_threads_);
//     return 1;
//   }
// }

// int RDMA::stop_all_listening_threads() {
//   for (auto& thread : threads_) {
//     thread->stop_listening();
//   }

//   threads_.clear();
//   thread_cnt_ = 0;
//   return 0;
// }

int RDMA::connect(const IPv4& client_addr, ConnectionId c_id,
                  ConnectionFlowId c_flow_id) {
  return nic_->add_connection(c_id, client_addr, c_flow_id);
}

int RDMA::disconnect(ConnectionId c_id) { 
  return nic_->close_connection(c_id); 
}

int RDMA::run_perf_thread(Nic::NicPerfMask perf_mask,
                          void (*callback)(const std::vector<uint64_t>&)) {
  return nic_->run_perf_thread(perf_mask, callback);
}

void RDMA::set_lb(int lb) { nic_->set_lb(lb); }

int RDMA::make_qp() {
  if (qp_pool_cnt_ < max_qp_pool_size_) {
    qp_pool_.emplace_back(std::unique_ptr<QueuePair>(
        new QueuePair(nic_.get(), qp_pool_cnt_, qp_pool_cnt_)));
    ++qp_pool_cnt_;
    return qp_pool_cnt - 1;
  } else {
    FRPC_ERROR("Max number of queue pairs is reached: %zu\n",
               max_qp_pool_size_);
    return -1;
  }
}

void add_send_queue_entry(uint16_t queue_pair_num, volatile int* data_addr, uint32_t data_size){
  for (auto qp : qp_pool_) {
    if (qp.get_qp_num() == queue_pair_num) {
      qp.add_send_queue_entry(data_addr, data_size);
      return 0;
    }
  }
  return 1; 
}
void add_recv_queue_entry(uint16_t queue_pair_num, volatile int* data_addr, uint32_t data_size){
  for (auto qp : qp_pool_) {
    if (qp.get_qp_num() == queue_pair_num) {
      qp.add_recv_queue_entry(data_addr, data_size);
      return 0;
    }
  }
  return 1;
}

int RDMA::send(uint16_t queue_pair_num) {
  // Pass data to send
  // Add arguments that take in data
  for (auto qp : qp_pool_) {
    if (qp.get_qp_num() == queue_pair_num) {
      // perform send op only when send queue is nonempty
      assert(!qp.send_q.empty());
      qp.send();
      return 0;
    }
  }
  return 1;
}

int RDMA::recv(uint16_t queue_pair_num) {
  // Pass data to send
  // Add arguments that take in data
  for (auto qp : qp_pool_) {
    if (qp.get_qp_num() == queue_pair_num) {
      qp.recv();
      return 0;
    }
  }
  return 1;  // qp number does not exist
}

bool RDMA::is_data_avaiable(uint16_t queue_pair_num) {
  // Pass data to send
  // Add arguments that take in data
  for (auto qp : qp_pool_) {
    if (qp.get_qp_num() == queue_pair_num) {
      return qp.is_data_avaiable();
    }
  }
}


int RDMA::stop_recv(uint16_t queue_pair_num) {
  // Pass data to send
  // Add arguments that take in data
  for (auto qp : qp_pool_) {
    if (qp.get_qp_num() == queue_pair_num) {
      qp.stop_recv();
      return 0;
    }
  }
  return 1;  // qp number does not exist
}

// QP Class, define state machine (inside qp.cc)
// function to modify state
// QP has send, receive, completion queue
// create or send/receive queue entry
// send and receive functions
// on receiver side
// given a pointer, memory is allocated
// define a struct with data field and put everything in data

int RDMA::connect_qp(uint16_t queue_pair_num, ConnectionId c_id, const IPv4& server_addr, uint16_t remote_qp_num, uint16_t p_key, uint32_t q_key) {
  for (auto qp : qp_pool_) {
    if (qp.get_qp_num() == queue_pair_num) {
      qp.connect(c_id, server_addr, remote_qp_num, p_key, q_key);
      return 0;
    }
  }
}

}  // namespace dagger
