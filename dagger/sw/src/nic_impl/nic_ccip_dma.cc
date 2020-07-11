#include "nic_ccip_dma.h"

#include <assert.h>

#include <opae/fpga.h>

#include "config.h"
#include "logger.h"

#include <iostream>

namespace frpc {

NicDmaCCIP::NicDmaCCIP(uint64_t base_nic_addr, size_t num_of_flows, bool master_nic = true):
    NicCCIP(base_nic_addr, master_nic),
    dp_configured_(false),
    num_of_flows_(num_of_flows),
    buf_(nullptr),
    tx_cl_offset_(0),
    rx_cl_offset_(0) {

}

NicDmaCCIP::~NicDmaCCIP() {
    if (dp_configured_) {
        fpgaReleaseBuffer(accel_handle_, wsid_);
        FRPC_INFO("Nic buffers are released\n");
    }
}

int NicDmaCCIP::configure_data_plane() {
    assert(connected_ == true);
    assert(initialized_ == true);
    assert(dp_configured_ == false);

    // Check the nic is dma-compatible
    uint64_t ccip_mode = 0;
    fpga_result res = fpgaReadMMIO64(accel_handle_,
                                     0,
                                     base_nic_addr_ + iRegCcipMode,
                                     &ccip_mode);
    if (res != FPGA_OK) {
        FRPC_ERROR("Nic configuration error, failed to read ccip mode register"
                    "nic returned: %d\n", res);
        return 1;
    }
    if (ccip_mode != iConstCcipDma) {
        FRPC_ERROR("Nic configuration error, "
                   "the harwdare is not CCI-P DMA compatible\n");
        return 1;
    }

    // Allocate Rx and Tx buffers
    size_t buff_size_bytes = 2 * num_of_flows_ * get_mtu_size_bytes();
    buf_ = (volatile char*)alloc_buffer(accel_handle_,
                                        buff_size_bytes,
                                        &wsid_,
                                        &buf_pa_);
    if (buf_ == nullptr) {
        FRPC_ERROR("Failed to allocate shared buffer\n");
        return 1;
    }
    FRPC_INFO("Shared nic buffer of size %uB is allocated by address 0x%x; "
              "buffer's nic-viewed physical address is 0x%x\n",
              buff_size_bytes,
              reinterpret_cast<volatile void*>(buf_),
              buf_pa_);

    // Configure Rx and Tx buffers
    // NIC's side data are cache line aligned
    tx_cl_offset_ = 0;
    rx_cl_offset_ = tx_cl_offset_ + num_of_flows_;

    res = fpgaWriteMMIO64(accel_handle_,
                          0,
                          base_nic_addr_ + iRegMemTxAddr,
                          buf_pa_ / CL(1) + tx_cl_offset_);
    if (res != FPGA_OK) {
        FRPC_ERROR("Nic configuration error, failed to configure Tx buffer,"
                    "nic returned %d\n", res);
        return 1;
    }

    res = fpgaWriteMMIO64(accel_handle_,
                          0,
                          base_nic_addr_ + iRegMemRxAddr,
                          buf_pa_ / CL(1) + rx_cl_offset_);
    if (res != FPGA_OK) {
        FRPC_ERROR("Nic configuration error, failed to configure Rx buffer,"
                    "nic returned %d\n", res);
        return 1;
    }

    // Configure the number of flows
    res = fpgaWriteMMIO64(accel_handle_,
                          0,
                          base_nic_addr_ + iRegNumOfFlows,
                          num_of_flows_);
    if (res != FPGA_OK) {
        FRPC_ERROR("Nic configuration error, failed to configure number of flows,"
                    "nic returned %d\n", res);
        return 1;
    }

    dp_configured_ = true;
    FRPC_INFO("Nic dataplane is configured\n");
    return 0;
}

int NicDmaCCIP::notify_nic_of_new_dma(size_t flow) const {
    dma_notification_lock_.lock();
    int res = fpgaWriteMMIO64(accel_handle_,
                              0,
                              base_nic_addr_ + iRegCcipDmaTrg,
                              flow);
    if (res != FPGA_OK) {
        FRPC_ERROR("Nic DMA notification error, nic returned %d\n", res);
        return 1;
    }
    dma_notification_lock_.unlock();

    return 0;
}

int NicDmaCCIP::start(bool perf) {
    assert(dp_configured_ == true);
    return start_nic(perf);
}

int NicDmaCCIP::stop() {
    assert(dp_configured_ == true);
    return stop_nic();
}

}  // namespace frpc