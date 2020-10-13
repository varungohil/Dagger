#include "rpc_server_thread.h"

#include <assert.h>

#include "config.h"
#include "rpc_header.h"
#include "logger.h"

#include <immintrin.h>

#include <unistd.h>

#include <iostream>

namespace frpc {

RpcServerThread::RpcServerThread(const Nic* nic,
                                 size_t nic_flow_id,
                                 uint16_t thread_id,
                                 const std::vector<const void*>& rpc_fn_ptr):
        thread_id_(thread_id),
        nic_(nic),
        nic_flow_id_(nic_flow_id),
        rpc_fn_ptr_(rpc_fn_ptr) {
#ifdef NIC_CCIP_MMIO
    if (cfg::nic::l_tx_queue_size != 0) {
        FRPC_ERROR("In MMIO mode, only one entry in the tx queue is allowed\n");
    }
#endif

    // Allocate queues
    tx_queue_ = TxQueue(nic_->get_tx_flow_buffer(nic_flow_id_),
                        nic_->get_mtu_size_bytes(),
                        cfg::nic::l_tx_queue_size);
    tx_queue_.init();

    rx_queue_ = RxQueue(nic_->get_rx_flow_buffer(nic_flow_id_),
                        nic_->get_mtu_size_bytes(),
                        cfg::nic::l_rx_queue_size);
    rx_queue_.init();

#ifdef NIC_CCIP_DMA
    current_batch_ptr = 0;
    batch_counter = 0;
#endif

    FRPC_INFO("Thread %d is created\n", thread_id_);
}

RpcServerThread::~RpcServerThread() {

}

void RpcServerThread::start_listening() {
    stop_signal_ = 0;
    thread_ = std::thread(&RpcServerThread::_PullListen, this);
}

void RpcServerThread::stop_listening() {
    stop_signal_ = 1;
    thread_.join();
}

// Pull-based listening
void RpcServerThread::_PullListen() {
    FRPC_INFO("Thread %d is listening now...\n", thread_id_);

    //if (rx_buff_ >= nic_->get_rx_buff_end()) {
    //    FRPC_ERROR("Nic rx buffer overflow \n");
    //    assert(false);
    //}

    constexpr size_t batch_size = 1 << cfg::nic::l_rx_batch_size;

    RpcPckt* req_pckt;

    while(!stop_signal_) {
        RpcPckt req_pckt_1[batch_size] __attribute__ ((aligned (64)));
        for(int i=0; i<batch_size && !stop_signal_; ++i) {
            // wait response
            uint32_t rx_rpc_id;
            req_pckt = reinterpret_cast<RpcPckt*>(rx_queue_.get_read_ptr(rx_rpc_id));
            while((req_pckt->hdr.ctl.valid == 0 ||
                   req_pckt->hdr.rpc_id == rx_rpc_id) &&
                  !stop_signal_) {
            }

            if (stop_signal_) continue;

            rx_queue_.update_rpc_id(req_pckt->hdr.rpc_id);

            req_pckt_1[i] = *req_pckt;

            //_mm256_store_si256(&req_pckt_1[i], *(reinterpret_cast<__m256i*>(req_pckt)));
            //_mm256_store_si256(reinterpret_cast<__m256i*>(&req_pckt_1[i] + 32),
            //                   *(reinterpret_cast<__m256i*>(req_pckt + 32)));

        }
        if (stop_signal_) continue;

        for(int i=0; i<batch_size; ++i) {
            // call function
            uint32_t ret_value = 0;
            if (req_pckt_1[i].hdr.fn_id == 0) {
                ret_value = (*reinterpret_cast<uint32_t(*)(uint32_t, uint32_t)>
                            (rpc_fn_ptr_[0]))(*reinterpret_cast<uint32_t*>(req_pckt_1[i].argv),
                                              *reinterpret_cast<uint32_t*>(req_pckt_1[i].argv + sizeof(uint32_t)));
            } else if (req_pckt_1[i].hdr.fn_id == 1) {
                ret_value = (*reinterpret_cast<uint32_t(*)(uint32_t)>
                            (rpc_fn_ptr_[1]))(*reinterpret_cast<uint32_t*>(req_pckt_1[i].argv));
            } else {
                FRPC_ERROR("Thread %d received a wrong function_id= %d\n", thread_id_, req_pckt_1[i].hdr.fn_id);
                // TODO: what should we do in this case?
            }

            // Get current buffer pointer
            uint8_t change_bit;
            char* tx_ptr = tx_queue_.get_write_ptr(change_bit);

            // return value
#ifdef NIC_CCIP_POLLING
            RpcPckt* tx_ptr_casted = reinterpret_cast<RpcPckt*>(tx_ptr);

            tx_ptr_casted->hdr.rpc_id      = req_pckt_1[i].hdr.rpc_id;
            tx_ptr_casted->hdr.n_of_frames = 1;
            tx_ptr_casted->hdr.frame_id    = 0;

            tx_ptr_casted->hdr.fn_id = req_pckt_1[i].hdr.fn_id;
            tx_ptr_casted->hdr.argl  = 1;

            tx_ptr_casted->hdr.ctl.req_type    = rpc_response;
            tx_ptr_casted->hdr.ctl.update_flag = change_bit;

            // Make data layout
            *(reinterpret_cast<uint32_t*>(tx_ptr_casted->argv)) = ret_value;

            _mm_mfence();
            tx_ptr_casted->hdr.ctl.valid = 1;
#elif NIC_CCIP_MMIO
            RpcPckt response __attribute__ ((aligned (64)));

            response.hdr.rpc_id      = req_pckt_1[i].hdr.rpc_id;
            response.hdr.n_of_frames = 1;
            response.hdr.frame_id    = 0;

            request.hdr.fn_id = req_pckt_1[i].hdr.fn_id;
            request.hdr.argl  = 1;

            request.hdr.ctl.req_type = rpc_response;
            request.hdr.ctl.valid    = 1;

            // Make data layout
            *(reinterpret_cast<uint32_t*>(response.argv)) = ret_value;

            // MMIO only supports AVX writes
            _mm256_store_si256(reinterpret_cast<__m256i*>(tx_ptr),
                               *(reinterpret_cast<__m256i*>(&response)));
            _mm256_store_si256(reinterpret_cast<__m256i*>(tx_ptr + 32),
                               *(reinterpret_cast<__m256i*>(&response)));
            _mm_mfence();
#elif NIC_CCIP_DMA
            RpcPckt* tx_ptr_casted = reinterpret_cast<RpcPckt*>(tx_ptr);

            tx_ptr_casted->hdr.rpc_id      = req_pckt_1[i].hdr.rpc_id;
            tx_ptr_casted->hdr.n_of_frames = 1;
            tx_ptr_casted->hdr.frame_id    = 0;

            tx_ptr_casted->hdr.fn_id = req_pckt_1[i].hdr.fn_id;
            tx_ptr_casted->hdr.argl  = 1;

            tx_ptr_casted->hdr.ctl.req_type    = rpc_response;
            tx_ptr_casted->hdr.ctl.update_flag = change_bit;

            // Make data layout
            *(reinterpret_cast<uint32_t*>(tx_ptr_casted->argv)) = ret_value;

            tx_ptr_casted->hdr.ctl.valid = 1;
            _mm_mfence();

            if (batch_counter == cfg::nic::tx_batch_size - 1) {
                nic_->notify_nic_of_new_dma(nic_flow_id_, current_batch_ptr);

                current_batch_ptr += cfg::nic::tx_batch_size;
                if (current_batch_ptr == ((1 << cfg::nic::l_tx_queue_size) / cfg::nic::tx_batch_size)*cfg::nic::tx_batch_size) {
                    current_batch_ptr = 0;
                }

                batch_counter = 0;
            } else {
                ++batch_counter;
            }
#else
    #error NIC CCI-P mode is not defined
#endif

        }
    }

    FRPC_INFO("Thread %d is stopped\n", thread_id_);
}

}  // namespace frpc
