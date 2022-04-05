#include "queuepair_v2.h"

#include <assert.h>
#include <immintrin.h>
#include <unistd.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "config.h"
#include "logger.h"
#include "rpc_header.h"

namespace dagger {

QueuePairV2::QueuePairV2(const Nic* nic, size_t nic_flow_id,
                         uint16_t queue_pair_num)
    : nic_(nic),
      nic_flow_id_(nic_flow_id),
      queue_pair_num_(queue_pair_num) {
#ifdef NIC_CCIP_MMIO
  if (cfg::nic::l_tx_queue_size != 0) {
    FRPC_ERROR("In MMIO mode, only one entry in the tx queue is allowed\n");
  }
#endif

  // Allocate queues in the nic.
  tx_queue_ = TxQueue(nic_->get_tx_flow_buffer(nic_flow_id_),
                      nic_->get_mtu_size_bytes(), cfg::nic::l_tx_queue_size);
  tx_queue_.init();

  rx_queue_ = RxQueue(nic_->get_rx_flow_buffer(nic_flow_id_),
                      nic_->get_mtu_size_bytes(), cfg::nic::l_rx_queue_size);
  rx_queue_.init();

  // cq_ = std::unique_ptr<CompletionQueue>(
  //     new CompletionQueue(nic_flow_id,
  //     nic_->get_rx_flow_buffer(nic_flow_id_),
  //                         nic_->get_mtu_size_bytes()));
  // cq_->bind();

#ifdef NIC_CCIP_DMA
  current_batch_ptr = 0;
  batch_counter = 0;
#endif
}

QueuePairV2::~QueuePairV2() {  }

// CompletionQueue* QueuePairV2::get_completion_queue() const { return
// cq_.get(); }

int QueuePairV2::connect(ConnectionId c_id, const IPv4& server_addr, uint16_t remote_qp_num, uint16_t p_key, uint32_t q_key) {
  remote_qp_num_ = remote_qp_num;
  p_key_ = p_key;
  q_key_ = q_key;
  return nic_->add_connection(c_id, server_addr, nic_flow_id_, remote_qp_num, p_key, q_key);
}

int QueuePairV2::disconnect(ConnectionId c_id) {
  return nic_->close_connection(c_id);
}

int QueuePairV2::start_listening() {
  stop_signal_ = 0;
  thread_ = std::thread(&QueuePairV2::_PullListen, this);

  // Pin thread to a certain CPU core
  // if (pin_cpu != -1) {
  //   cpu_set_t cpuset;
  //   CPU_ZERO(&cpuset);
  //   CPU_SET(pin_cpu, &cpuset);
  //   int rc = pthread_setaffinity_np(thread_.native_handle(),
  //   sizeof(cpu_set_t),
  //                                   &cpuset);
  //   if (rc != 0) {
  //     FRPC_ERROR("Failed to pin thread %d to CPU %d\n", thread_id_, pin_cpu);
  //     return 1;
  //   }
  // }

  return 0;
}

void QueuePairV2::stop_listening() {
  stop_signal_ = 1;
  thread_.join();
}

// Pull-based listening on the dispatch thread.
void QueuePairV2::_PullListen() {
  // FRPC_INFO("Thread %d is listening now on CPU %d\n", thread_id_,
  //           sched_getcpu());
  // if (rx_buff_ >= nic_->get_rx_buff_end()) {
  //    FRPC_ERROR("Nic rx buffer overflow \n");
  //    assert(false);
  //}

  constexpr size_t batch_size = 1 << cfg::nic::l_rx_batch_size;

  volatile RpcPckt* req_pckt;

  while (!stop_signal_) {
    RpcPckt req_pckt_1[batch_size] __attribute__((aligned(64)));
    for (int i = 0; i < batch_size && !stop_signal_; ++i) {
      // wait response
      uint32_t rx_rpc_id;
      req_pckt = reinterpret_cast<volatile RpcPckt*>(
          rx_queue_.get_read_ptr(rx_rpc_id));
      while (
          (req_pckt->hdr.ctl.valid == 0 || req_pckt->hdr.rpc_id == rx_rpc_id) &&
          !stop_signal_) {
      }

      if (stop_signal_) continue;

      rx_queue_.update_rpc_id(req_pckt->hdr.rpc_id);

      req_pckt_1[i] = *const_cast<RpcPckt*>(req_pckt);

      //_mm256_store_si256(&req_pckt_1[i],
      //*(reinterpret_cast<__m256i*>(req_pckt)));
      //_mm256_store_si256(reinterpret_cast<__m256i*>(&req_pckt_1[i] + 32),
      //                   *(reinterpret_cast<__m256i*>(req_pckt + 32)));
    }
    if (stop_signal_) continue;

    for (int i = 0; i < batch_size; ++i) {
      //    std::cout << "DEBUG: recv packet: ********* "
      //        << "\n hdr.ctl.req_type: " << (int)((req_pckt_1 +
      //        i)->hdr.ctl.req_type)
      //        << "\n hdr.ctl.valid: " << (int)((req_pckt_1 +
      //        i)->hdr.ctl.valid)
      //        << "\n hdr.argl: " << (int)((req_pckt_1 + i)->hdr.argl)
      //        << "\n hdr.c_id: " << (int)((req_pckt_1 + i)->hdr.c_id)
      //        << "\n hdr.rpc_id: " << (int)((req_pckt_1 + i)->hdr.rpc_id)
      //        << "\n hdr.fn_id: " << (int)((req_pckt_1 + i)->hdr.fn_id)
      //        << "\n argv: ";
      //    for (int j=0; j<cfg::sys::cl_size_bytes - rpc_header_size_bytes;
      //    ++j) {
      //        std::cout << (int)(((req_pckt_1 + i)->argv)[j]) << " ";
      //    }
      //    std::cout << "\n **************** " << std::endl;

      operator()(req_pckt_1 + i, tx_queue_);
    }
  }

  // FRPC_INFO("Thread %d is stopped\n", thread_id_);
}

// push data_addr onto queue q
// void QueuePairV2::append_elem(std::queue<QueueElem> q,
//                               volatile char* data_addr) {
//   QueueElem next_req;
//   next_req.data_addr = data_addr;
//   q.push(next_req);
// }

void QueuePairV2::add_send_queue_entry(volatile char* data_addr) {
  QueueElem new_entry;
  new_entry.data_addr = data_addr;
  send_q.push(new_entry);
}

void QueuePairV2::add_recv_queue_entry(volatile char* data_addr) {
  QueueElem new_entry;
  new_entry.data_addr = data_addr;
  recv_q.push(new_entry);
}

// put entry in send queue, put entry in receive queue
// need a function to put an entry in send queue
// delete entry in operator (.h)
// 1. call function to put entry in send queue
// 2. call send (precondition: there's an element in send queue)
// 3. check if entry is in receive queue or raise error
// data written to hardware tx_buffer not send

int QueuePairV2::send() {
  // Get current buffer pointer
  uint8_t change_bit;
  volatile char* tx_ptr = tx_queue_.get_write_ptr(change_bit);
  if (tx_ptr >= nic_->get_tx_buff_end()) {
    // FRPC_ERROR("Nic tx buffer overflow \n");
    assert(false);
  }
  assert(reinterpret_cast<size_t>(tx_ptr) % nic_->get_mtu_size_bytes() == 0);

  // Make RPC id
  // uint32_t rpc_id = client_id_ | static_cast<uint32_t>(rpc_id_cnt_ << 16);

  uint32_t rpc_id = 0;

  QueueElem entry = send_q.front();
  send_q.pop();
  GetRequest args = *(reinterpret_cast<GetRequest*>(const_cast<char*>(entry.data_addr)));
  // reinterpret_cast<GetRequest*>(const_cast<char*>(entry->data_addr)) = args;


  // // call operator to do fulfill send 
  // operator()(req_pckt, tx_queue_); ?? Raleigh did you add this?

  // read first entry of send_q
  //  Send data
#ifdef NIC_CCIP_POLLING
  volatile RpcPckt* tx_ptr_casted = reinterpret_cast<volatile RpcPckt*>(tx_ptr);

  tx_ptr_casted->hdr.c_id = c_id_;
  tx_ptr_casted->hdr.rpc_id = rpc_id;
  tx_ptr_casted->hdr.n_of_frames = 1;
  tx_ptr_casted->hdr.frame_id = 0;

  tx_ptr_casted->hdr.fn_id = 0;
  tx_ptr_casted->hdr.argl = sizeof(GetRequest);

  tx_ptr_casted->hdr.ctl.req_type = rpc_request;
  tx_ptr_casted->hdr.ctl.update_flag = change_bit;

  *reinterpret_cast<GetRequest*>(const_cast<uint8_t*>(tx_ptr_casted->argv)) =
      args;

  // Set valid
  _mm_mfence();
  tx_ptr_casted->hdr.ctl.valid = 1;
#elif NIC_CCIP_MMIO
  RpcPckt request __attribute__((aligned(64)));

  request.hdr.c_id = c_id_;
  request.hdr.rpc_id = rpc_id;
  request.hdr.n_of_frames = 1;
  request.hdr.frame_id = 0;

  request.hdr.fn_id = 0;
  request.hdr.argl = sizeof(GetRequest);

  request.hdr.ctl.req_type = rpc_request;
  request.hdr.ctl.valid = 1;

  _mm_mfence();

  memcpy(request.argv, reinterpret_cast<const void*>(&args),
         sizeof(GetRequest));

// MMIO only supports AVX writes
#  ifdef PLATFORM_PAC_A10
  // PAC_A10 supports AVX-512 - easy!
  _mm512_store_si512(reinterpret_cast<__m512i*>(tx_ptr),
                     *(reinterpret_cast<__m512i*>(&request)));
#  else
  // BDX only supports AVX-256, so split into two writes
  //  - performance will not be good
  //  - and I'm not even sure, this will ever work (so far, I have not seen any
  //  testing issues)
  //  - better to avoid the MMIO interface for BDX
  _mm256_store_si256(reinterpret_cast<__m256i*>(tx_ptr),
                     *(reinterpret_cast<__m256i*>(&request)));
  _mm256_store_si256(
      reinterpret_cast<__m256i*>(tx_ptr + 32),
      *(reinterpret_cast<__m256i*>(reinterpret_cast<uint8_t*>(&request) + 32)));
#  endif
#elif NIC_CCIP_DMA
  RpcPckt* tx_ptr_casted = reinterpret_cast<RpcPckt*>(tx_ptr);

  tx_ptr_casted->hdr.c_id = c_id_;
  tx_ptr_casted->hdr.rpc_id = rpc_id;
  tx_ptr_casted->hdr.n_of_frames = 1;
  tx_ptr_casted->hdr.frame_id = 0;

  tx_ptr_casted->hdr.fn_id = 0;
  tx_ptr_casted->hdr.argl = sizeof(GetRequest);

  tx_ptr_casted->hdr.ctl.req_type = rpc_request;
  tx_ptr_casted->hdr.ctl.update_flag = change_bit;

  *reinterpret_cast<GetRequest*>(const_cast<uint8_t*>(tx_ptr_casted->argv)) =
      args;

  tx_ptr_casted->hdr.ctl.valid = 1;
  _mm_mfence();

  if (batch_counter == cfg::nic::tx_batch_size - 1) {
    nic_->notify_nic_of_new_dma(nic_flow_id_, current_batch_ptr);

    current_batch_ptr += cfg::nic::tx_batch_size;
    if (current_batch_ptr ==
        ((1 << cfg::nic::l_tx_queue_size) / cfg::nic::tx_batch_size) *
            cfg::nic::tx_batch_size) {
      current_batch_ptr = 0;
    }

    batch_counter = 0;
  } else {
    ++batch_counter;
  }
#else
#  error NIC CCI-P mode is not defined
#endif

  // ++rpc_id_cnt_;

  return 0;
}

int QueuePairV2::recv() {
  start_listening();
  return 0;
}

int QueuePairV2::stop_recv() {
  stop_listening();
  return 0;
}

uint16_t QueuePairV2::get_qp_num() { 
  return queue_pair_num_;
}

}  // namespace dagger
