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
#include <string>
#include <iostream>
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

  server_callback_ = std::unique_ptr<dagger::RdmaServerCallBack>(new dagger::RdmaServerCallBack());
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

void QueuePairV2::operator_(const RpcPckt* rpc_in, TxQueue& tx_queue) {
  // uint8_t ret_buff[cfg::sys::cl_size_bytes];

  // Check the fn_id is within the scope
  // if (rpc_in->hdr.fn_id > rpc_fn_ptr_.size() - 1) {
  //   FRPC_ERROR(
  //       "Too large RPC function id is received, this call will stop here
  //       and " "no value will be returned\n");
  //   return;
  // }

  // read first received packet
  QueueElem entry = recv_q.front();
  recv_q.pop();
  std::cout << "Receive:: pop : Recv queue len now = " << recv_q.size() << std::endl;
  std::cout << "Receive:: rpc_in->argv = " << rpc_in->argv << std::endl;
  std::cout << "Receive:: rpc_in->hdr.rpc_id = " << rpc_in->hdr.rpc_id << std::endl;
  std::cout << "Receive:: entry.data_addr = " << (int*)(entry.data_addr) << std::endl;
  *(const_cast<int*>(entry.data_addr)) = *(reinterpret_cast<int*>(const_cast<uint8_t*>(rpc_in->argv)));
  //*(const_cast<int*>(entry.data_addr)) = rpc_in->argv;
  std::cout << "Receive:: Value at addr" << (int)(*(entry.data_addr)) << std::endl;


  // // set store addr based on request argv
  // r_elem.data_addr = (int*)rpc_in->argv;

  //     uint8_t change_bit;
  //     volatile int* tx_ptr = tx_queue.get_write_ptr(change_bit);

  //     // Send data
  // #ifdef NIC_CCIP_POLLING
  //     volatile RpcPckt* tx_ptr_casted =
  //         reinterpret_cast<volatile RpcPckt*>(tx_ptr);

  //     tx_ptr_casted->hdr.c_id = rpc_in->hdr.c_id;
  //     tx_ptr_casted->hdr.rpc_id = rpc_in->hdr.rpc_id;
  //     tx_ptr_casted->hdr.n_of_frames = 1;
  //     tx_ptr_casted->hdr.frame_id = 0;

  //     tx_ptr_casted->hdr.fn_id = 1;
  //     tx_ptr_casted->hdr.argl = ret_size;

  //     tx_ptr_casted->hdr.ctl.req_type = rpc_response;
  //     tx_ptr_casted->hdr.ctl.update_flag = change_bit;

  //     memcpy(const_cast<uint8_t*>(tx_ptr_casted->argv), ret_buff,
  //     ret_size);

  //     // Set valid
  //     _mm_mfence();
  //     tx_ptr_casted->hdr.ctl.valid = 1;
  // #elif NIC_CCIP_MMIO
  //     RpcPckt request __attribute__((aligned(64)));

  //     request.hdr.c_id = rpc_in->hdr.c_id;
  //     request.hdr.rpc_id = rpc_in->hdr.rpc_id;
  //     request.hdr.n_of_frames = 1;
  //     request.hdr.frame_id = 0;

  //     request.hdr.fn_id = 1;
  //     request.hdr.argl = ret_size;

  //     request.hdr.ctl.req_type = rpc_response;
  //     request.hdr.ctl.valid = 1;

  //     _mm_mfence();

  //     memcpy(request.argv, ret_buff, ret_size);

  // // MMIO only supports AVX writes
  // #  ifdef PLATFORM_PAC_A10
  //     // PAC_A10 supports AVX-512 - easy!
  //     _mm512_store_si512(reinterpret_cast<__m512i*>(tx_ptr),
  //                        *(reinterpret_cast<__m512i*>(&request)));
  // #  else
  //     // BDX only supports AVX-256, so split into two writes
  //     //  - performance will not be good
  //     //  - and I'm not even sure, this will ever work (so far, I have not
  //     seen
  //     //  any testing issues)
  //     //  - better to avoid the MMIO interface for BDX
  //     _mm256_store_si256(reinterpret_cast<__m256i*>(tx_ptr),
  //                        *(reinterpret_cast<__m256i*>(&request)));
  //     _mm256_store_si256(reinterpret_cast<__m256i*>(tx_ptr + 32),
  //                        *(reinterpret_cast<__m256i*>(
  //                            reinterpret_cast<uint8_t*>(&request) + 32)));
  // #  endif
  // #elif NIC_CCIP_DMA
  //     RpcPckt* tx_ptr_casted = reinterpret_cast<RpcPckt*>(tx_ptr);

  //     tx_ptr_casted->hdr.c_id = rpc_in->hdr.c_id;
  //     tx_ptr_casted->hdr.rpc_id = rpc_in->hdr.rpc_id;
  //     tx_ptr_casted->hdr.n_of_frames = 1;
  //     tx_ptr_casted->hdr.frame_id = 0;

  //     tx_ptr_casted->hdr.fn_id = 1;
  //     tx_ptr_casted->hdr.argl = ret_size;

  //     tx_ptr_casted->hdr.ctl.req_type = rpc_response;
  //     tx_ptr_casted->hdr.ctl.update_flag = change_bit;

  //     memcpy(const_cast<uint8_t*>(tx_ptr_casted->argv), ret_buff,
  //     ret_size);

  //     tx_ptr_casted->hdr.ctl.valid = 1;
  //     _mm_mfence();

  //     if (batch_counter == cfg::nic::tx_batch_size - 1) {
  //       nic_->notify_nic_of_new_dma(nic_flow_id_, current_batch_ptr);

  //       current_batch_ptr += cfg::nic::tx_batch_size;
  //       if (current_batch_ptr ==
  //           ((1 << cfg::nic::l_tx_queue_size) / cfg::nic::tx_batch_size) *
  //               cfg::nic::tx_batch_size) {
  //         current_batch_ptr = 0;
  //       }

  //       batch_counter = 0;
  //     } else {
  //       ++batch_counter;
  //     }
  // #else
  // #  error NIC CCI-P mode is not defined
  // #endif
  //   }

  // #ifdef NIC_CCIP_DMA
  //   uint32_t current_batch_ptr;
  //   size_t batch_counter;
  // #endif
}

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
  //thread_2 = std::thread(&QueuePairV2::_PullListen, this);

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

bool QueuePairV2::is_data_available() {
  return is_data_available_;
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
      std::cout << "Receive:: Going Inside spin loop!" << std::endl;
      std::cout << "Recieve:: Before : Valid = " << (int)(req_pckt->hdr.ctl.valid) << "; ReqHdr RPC ID = " << req_pckt->hdr.rpc_id << "; RxRPC ID = " << rx_rpc_id << "; Stop Signal = "<< stop_signal_ << "; argv = " << req_pckt->argv << std::endl;
      //int count = 0;
      while (
          (req_pckt->hdr.ctl.valid == 0 || req_pckt->hdr.rpc_id == rx_rpc_id) &&
          !stop_signal_) {
      //if(count%100 == 0){
      //std::cout << "Loop: Valid = " << (int)(req_pckt->hdr.ctl.valid) << "; ReqHdr RPC ID = " << req_pckt->hdr.rpc_id << "; RxRPC ID = " << rx_rpc_id << "; Stop signal = " << stop_signal_ << std::endl;
      //std::cout << "valid = " << (int)(req_pckt->hdr.ctl.valid) << " HDR RPCID = " << req_pckt->hdr.rpc_id << " RX RPCID = " << rx_rpc_id << std::endl;
       //}
      //count++;
      }
      std::cout << "Receive:: Valid = " << (int)(req_pckt->hdr.ctl.valid) << "; ReqHdr RPC ID = " << req_pckt->hdr.rpc_id << "; RxRPC ID = " << rx_rpc_id << "; Stop signal = " << stop_signal_ << "; argv = " << req_pckt->argv <<  std::endl;
      std::cout << "Receive:: Got out of spin loop" << std::endl;
      //std::cout << "Receive:: req_pckt->argv = " << req_pckt->argv << std::endl;
      if (stop_signal_) continue;

      rx_queue_.update_rpc_id(req_pckt->hdr.rpc_id);

      req_pckt_1[i] = *const_cast<RpcPckt*>(req_pckt);
      //if(req_pckt->hdr.rpc_id == 0){
      // *reinterpret_cast<int*>(const_cast<uint8_t*>(req_pckt->argv)) = 19;
      // }
      // std::cout << "req_pckt : ";
      // for(int j = 0; j < cfg::sys::cl_size_bytes - rpc_header_size_bytes; ++j){
      //     std::cout << (int)(req_pckt->argv[j]) << " ";
      // }
      // std::cout << "\n";
      //_mm256_store_si256(&req_pckt_1[i],
      //*(reinterpret_cast<__m256i*>(req_pckt)));
      //_mm256_store_si256(reinterpret_cast<__m256i*>(&req_pckt_1[i] + 32),
      //                   *(reinterpret_cast<__m256i*>(req_pckt + 32)));
    }
    is_data_available_ = 1;
    if (stop_signal_) continue;

    for (int i = 0; i < batch_size; ++i) {
          std::cout << "DEBUG: recv packet: ********* "
              << "\n hdr.ctl.req_type: " << (int)((req_pckt_1 +
              i)->hdr.ctl.req_type)
              << "\n hdr.ctl.valid: " << (int)((req_pckt_1 +
              i)->hdr.ctl.valid)
              << "\n hdr.argl: " << (int)((req_pckt_1 + i)->hdr.argl)
              << "\n hdr.c_id: " << (int)((req_pckt_1 + i)->hdr.c_id)
              << "\n hdr.rpc_id: " << (int)((req_pckt_1 + i)->hdr.rpc_id)
              << "\n hdr.fn_id: " << (int)((req_pckt_1 + i)->hdr.fn_id) 
             // << "\n argv: " << (req_pckt_1 + i)->argv 
              //<< std::endl;
              << "\n argv: ";
          for (int j=0; j<cfg::sys::cl_size_bytes - rpc_header_size_bytes;
          ++j) {
              std::cout << (int)((req_pckt_1 + i)->argv)[j] << " ";
          }
          std::cout << "\n **************** " << std::endl;
      //sleep(1);
      // QueueElem entry = recv_q.front();
      // recv_q.pop();
      //std::cout << "Receive:: pop : Recv queue len now = " << recv_q.size() << std::endl;
      //std::cout << "Receive:: rpc_in->argv = " << (int)(rpc_in->argv) << std::endl;
      //std::cout << "Receive:: rpc_in->hdr.rpc_id = " << rpc_in->hdr.rpc_id << std::endl;
      //std::cout << "Receive:: entry.data_addr = " << (int*)(entry.data_addr) << std::endl;
      server_callback_->operator()(req_pckt_1 + i, entry);
      // *(const_cast<int*>(entry.data_addr)) = *(reinterpret_cast<int*>(const_cast<uint8_t*>((req_pckt_1 + i)->argv)));
      //*(const_cast<int*>(entry.data_addr)) = rpc_in->argv;
      //std::cout << "Receive:: Value at addr" << (int)(*(entry.data_addr)) << std::endl;
      // operator_(req_pckt_1 + i, tx_queue_);
     //
    }
    is_data_available_ = 0;
  }

  // FRPC_INFO("Thread %d is stopped\n", thread_id_);
}

// push data_addr onto queue q
// void QueuePairV2::append_elem(std::queue<QueueElem> q,
//                               volatile int* data_addr) {
//   QueueElem next_req;
//   next_req.data_addr = data_addr;
//   q.push(next_req);
// }

void QueuePairV2::add_send_queue_entry(volatile int* data_addr, size_t data_size) {
  QueueElem new_entry;
  new_entry.data_addr = data_addr;
  new_entry.data_size = data_size;
  send_q.push(new_entry);
}

void QueuePairV2::add_recv_queue_entry(volatile int* data_addr, size_t data_size) {
  QueueElem new_entry;
  new_entry.data_addr = data_addr;
  new_entry.data_size = data_size;
  recv_q.push(new_entry);
  std::cout << "Receive:: Recv Queue Len = "<< recv_q.size() << std::endl;
}

// put entry in send queue, put entry in receive queue
// need a function to put an entry in send queue
// delete entry in operator (.h)
// 1. call function to put entry in send queue
// 2. call send (precondition: there's an element in send queue)
// 3. check if entry is in receive queue or raise error
// data written to hardware tx_buffer not send

int QueuePairV2::send() {
  //std::cout << "In QP::send()" << std::endl;
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

  uint32_t rpc_id = rpc_id_count_;
  rpc_id_count_++;
  //std::cout << "Reading send queue entry" << std::endl;
  QueueElem entry = send_q.front();
  send_q.pop();
  //std::cout << "read send queue entry" << std::endl;
  // std::string args = *(const_cast<int*>(entry.data_addr));
  int args = *(const_cast<int*>(entry.data_addr));
  //std::cout << "Send:: Data in sendq entry = " << args << std::endl; 
  size_t data_size = entry.data_size;
  // reinterpret_cast<GetRequest*>(const_cast<int*>(entry->data_addr)) = args;
  //std::cout << "read datav at data addr" << std::endl;

  // // call operator to do fulfill send 
  // operator_()(req_pckt, tx_queue_); ?? Raleigh did you add this?

  // read first entry of send_q
  //  Send data
#ifdef NIC_CCIP_POLLING
  //std::cout << "In NIC_CCIP_POLLING" << std::endl;
  volatile RpcPckt* tx_ptr_casted = reinterpret_cast<volatile RpcPckt*>(tx_ptr);
  //std::cout << "TX_PTR_CASTED" << std::endl;
  tx_ptr_casted->hdr.c_id = c_id_;
  tx_ptr_casted->hdr.rpc_id = rpc_id;
  tx_ptr_casted->hdr.n_of_frames = 1;
  tx_ptr_casted->hdr.frame_id = 0;

  tx_ptr_casted->hdr.fn_id = rpc_id;
  tx_ptr_casted->hdr.argl = data_size;

  tx_ptr_casted->hdr.ctl.req_type = rpc_request;
  tx_ptr_casted->hdr.ctl.update_flag = change_bit;
  //std::cout << "Writing to argv" << std::endl;
  std::cout << "Send:: Args = " << args << std::endl;
  for(int j=0; j < cfg::sys::cl_size_bytes - rpc_header_size_bytes; j=j+4){
     *reinterpret_cast<volatile int*>(const_cast<uint8_t*>(tx_ptr_casted->argv + j)) = args;
  }
  //*reinterpret_cast<volatile int*>(const_cast<uint8_t*>(tx_ptr_casted->argv)) = args;
  //tx_ptr_casted->argv = argv;
  //tx_ptr_casted->argv =  args;
  std::cout << "Send::argv = " << (int)(*tx_ptr_casted->argv) << std::endl;
  for(int j=0; j < cfg::sys::cl_size_bytes - rpc_header_size_bytes; ++j){
      std::cout << (int)(tx_ptr_casted->argv[j]) << " ";
  }
  std::cout << "\n";
  std::cout << "Send::RPC_ID = " << tx_ptr_casted->hdr.rpc_id << std::endl;
  //std::cout << "Wrote to argv" << std::endl;

  // Set valid
  _mm_mfence();
  //std::cout << "Setting valid" << std::endl;
  tx_ptr_casted->hdr.ctl.valid = 1;
#elif NIC_CCIP_MMIO
  RpcPckt request __attribute__((aligned(64)));

  request.hdr.c_id = c_id_;
  request.hdr.rpc_id = rpc_id;
  request.hdr.n_of_frames = 1;
  request.hdr.frame_id = 0;

  // request.hdr.fn_id = 0;
  request.hdr.argl = data_size;

  request.hdr.ctl.req_type = rpc_request;
  request.hdr.ctl.valid = 1;

  _mm_mfence();

  memcpy(request.argv, reinterpret_cast<const void*>(&args),
         data_size);

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
  tx_ptr_casted->hdr.argl = data_size;

  tx_ptr_casted->hdr.ctl.req_type = rpc_request;
  tx_ptr_casted->hdr.ctl.update_flag = change_bit;

  *reinterpret_cast<volatile int*>(tx_ptr_casted->argv)) =
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
