
/*
 * Autogenerated with rpc_gen.py
 *
 *        DO NOT CHANGE
*/
#ifndef _RPC_CLIENT_NONBLOCKING_H_
#define _RPC_CLIENT_NONBLOCKING_H_

#include "logger.h"
#include "rpc_client_nonblocking_base.h"
#include "utils.h"

#include "rpc_types.h"

#include <cstring>
#include <immintrin.h>

namespace dagger {

class RpcClient: public RpcClientNonBlock_Base {
public:
    RpcClient(const Nic* nic, size_t nic_flow_id, uint16_t client_id):
        RpcClientNonBlock_Base(nic, nic_flow_id, client_id) {}
    virtual ~RpcClient() {}

    virtual void abstract_class() const { return; }

    // Remote function section
    int get(const GetRequest& args) {

        // Get current buffer pointer
        uint8_t change_bit;
        volatile char* tx_ptr = tx_queue_.get_write_ptr(change_bit);
        if (tx_ptr >= nic_->get_tx_buff_end()) {
            FRPC_ERROR("Nic tx buffer overflow \n");
            assert(false);
        }
        assert(reinterpret_cast<size_t>(tx_ptr) % nic_->get_mtu_size_bytes() == 0);

        // Make RPC id
        uint32_t rpc_id = client_id_ | static_cast<uint32_t>(rpc_id_cnt_ << 16);
        // Send data
    #ifdef NIC_CCIP_POLLING
        volatile RpcPckt* tx_ptr_casted = reinterpret_cast<volatile RpcPckt*>(tx_ptr);

        tx_ptr_casted->hdr.c_id        = c_id_;
        tx_ptr_casted->hdr.rpc_id      = rpc_id;
        tx_ptr_casted->hdr.n_of_frames = 1;
        tx_ptr_casted->hdr.frame_id    = 0;

        tx_ptr_casted->hdr.fn_id  = 0;
        tx_ptr_casted->hdr.argl   = sizeof(GetRequest);

        tx_ptr_casted->hdr.ctl.req_type    = rpc_request;
        tx_ptr_casted->hdr.ctl.update_flag = change_bit;

        *reinterpret_cast<GetRequest*>(const_cast<uint8_t*>(tx_ptr_casted->argv)) = args;

        // Set valid
        _mm_mfence();
        tx_ptr_casted->hdr.ctl.valid = 1;
    #elif NIC_CCIP_MMIO
        RpcPckt request __attribute__ ((aligned (64)));

        request.hdr.c_id        = c_id_;
        request.hdr.rpc_id      = rpc_id;
        request.hdr.n_of_frames = 1;
        request.hdr.frame_id    = 0;

        request.hdr.fn_id = 0;
        request.hdr.argl  = sizeof(GetRequest);

        request.hdr.ctl.req_type = rpc_request;
        request.hdr.ctl.valid    = 1;

        _mm_mfence();

        memcpy(request.argv, reinterpret_cast<const void*>(&args), sizeof(GetRequest));


        // MMIO only supports AVX writes
        #ifdef PLATFORM_PAC_A10
            // PAC_A10 supports AVX-512 - easy!
            _mm512_store_si512(reinterpret_cast<__m512i*>(tx_ptr),
                               *(reinterpret_cast<__m512i*>(&request)));
        #else
            // BDX only supports AVX-256, so split into two writes
            //  - performance will not be good
            //  - and I'm not even sure, this will ever work (so far, I have not seen any testing issues)
            //  - better to avoid the MMIO interface for BDX
            _mm256_store_si256(reinterpret_cast<__m256i*>(tx_ptr),
                               *(reinterpret_cast<__m256i*>(&request)));
            _mm256_store_si256(reinterpret_cast<__m256i*>(tx_ptr + 32),
                               *(reinterpret_cast<__m256i*>(reinterpret_cast<uint8_t*>(&request) + 32)));
        #endif
    #elif NIC_CCIP_DMA
        RpcPckt* tx_ptr_casted = reinterpret_cast<RpcPckt*>(tx_ptr);

        tx_ptr_casted->hdr.c_id        = c_id_;
        tx_ptr_casted->hdr.rpc_id      = rpc_id;
        tx_ptr_casted->hdr.n_of_frames = 1;
        tx_ptr_casted->hdr.frame_id    = 0;

        tx_ptr_casted->hdr.fn_id = 0;
        tx_ptr_casted->hdr.argl  = sizeof(GetRequest);

        tx_ptr_casted->hdr.ctl.req_type    = rpc_request;
        tx_ptr_casted->hdr.ctl.update_flag = change_bit;

        *reinterpret_cast<GetRequest*>(const_cast<uint8_t*>(tx_ptr_casted->argv)) = args;

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


        ++rpc_id_cnt_;

        return 0;
}

};

}  // namespace dagger

#endif // _RPC_CLIENT_NONBLOCKING_H_
