#ifndef _NIC_CCIP_MMIO_H_
#define _NIC_CCIP_MMIO_H_

#include <stddef.h>
#include <stdint.h>

#include "nic_ccip.h"

namespace frpc {

#define CL(x) ((x) * NicCCIP::cacheline_size_bytes)

/// Polling-based CCIP NIC.
/// Provides software support for CCI-P polling.
///
/// Inheritance hierarchy:
///   Nic -> NicCCIP -> NicPollingCCIP
///                  -> NicMmioCCIP
///                  -> NicDmaCCIP
///
class NicMmioCCIP: public NicCCIP {
public:
    NicMmioCCIP(uint64_t base_rf_addr, size_t num_of_flows, bool master_nic);
    virtual ~NicMmioCCIP();

    virtual int start(bool perf=false);
    virtual int stop();

    virtual int configure_data_plane();

    virtual int notify_nic_of_new_dma(size_t flow) const {
        // No needs to explicitly notify NIC
        return 0;
    }

    virtual char* get_tx_flow_buffer(size_t flow) const {
        return reinterpret_cast<char*>(tx_mmio_buf_) + tx_cl_offset_ + CL(flow);
    }

    virtual volatile char* get_rx_flow_buffer(size_t flow) const {
        return buf_ + CL(rx_cl_offset_) + CL(flow);
    }

    virtual const char* get_tx_buff_end() const { return nullptr; }
    virtual const char* get_rx_buff_end() const { return nullptr; }

private:
    bool dp_configured_;

    // Number of Nic flows;
    // one flow = one CPU-NIC communication channel
    size_t num_of_flows_;

    // Mmaped Rx buffer
    volatile char *buf_;
    // Buffer id
    uint64_t wsid_;
    // NIC-viewed physical address of the buffer
    uint64_t buf_pa_;
    // Offsets
    uint64_t rx_cl_offset_;

    // MMIO-maped Tx buffer;
    uint64_t* tx_mmio_buf_;
    // Offset
    uint64_t tx_cl_offset_;

};

}  // namespace frpc

#endif