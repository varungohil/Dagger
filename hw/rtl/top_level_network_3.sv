// Author: Cornell University
//
// Module Name :    top_level_network_module
// Project :        F-NIC
// Description :    Top-level module for the design
//                    - using real networking
//                    - instantiates one NIC and the networking MAC/PHY
//

`include "platform_if.vh"

`include "nic.sv"

//`include "nic_defs.vh"

module top_level_network_module (
    // CCI-P Clocks and Resets
    input           logic             pClk,          // 200MHz
    input           logic             pClkDiv2,      // 100MHz
    input           logic             pReset,

    // Host interface
    input           t_if_ccip_Rx      pck_cp2af_sRx,        // CCI-P Rx Port
    output          t_if_ccip_Tx      pck_af2cp_sTx,        // CCI-P Tx Port

    // Raw HSSI interface
    pr_hssi_if.to_fiu   hssi

    );

    // We need four sub-afus:
    // * 3 NICs
    // * Ethernet MAC/PHY
    localparam NUM_SUB_AFUS    = 4;
    localparam NUM_PIPE_STAGES = 2;


    // Define clock and reset for design
    //logic clk;
    //assign clk = pClkDiv2;

    logic clk;
    assign clk = pClk;

    logic clk_div_2;
    assign clk_div_2 = pClkDiv2;

    logic reset;
    assign reset = pReset;

    // Register requests
    t_if_ccip_Rx sRx;
    always_ff @(posedge clk)
    begin
        sRx <= pck_cp2af_sRx;
    end

    t_if_ccip_Tx sTx;
    assign pck_af2cp_sTx = sTx;


    // =============================================================
    // Install CCI-P MUX
    // =============================================================
    t_if_ccip_Rx    pck_afu_RxPort        [NUM_SUB_AFUS-1:0];
    t_if_ccip_Tx    pck_afu_TxPort        [NUM_SUB_AFUS-1:0];
    logic           ccip_mux2pe_reset     [NUM_SUB_AFUS-1:0];

    PhyAddr                        iRegPhyNetAddr;
    IPv4                           iRegIpv4NetAddr;

    ccip_mux #(NUM_SUB_AFUS, NUM_PIPE_STAGES) ccip_mux_U0 (
                        .pClk(clk),
                        .pClkDiv2(clk_div_2),
                        .SoftReset(reset),
                        .up_Error(),
                        .up_PwrState(),
                        .up_RxPort(sRx),
                        .up_TxPort(sTx),
                        .afu_SoftReset(ccip_mux2pe_reset),
                        .afu_PwrState(),
                        .afu_Error(),
                        .afu_RxPort(pck_afu_RxPort),
                        .afu_TxPort(pck_afu_TxPort)
        );

    // =============================================================
    // Instantiate a NIC device
    //   - as CCI-P device #1 since we want to use the lower CSR addresses for the
    //     Ethernet MAC/PHY
    //   - HW MMIO addr is in 32-bit space (1)
    //     HW MMIO objects are 64-bits (2)
    //     (1, 2) -> HW MMIO addresses are mult. of 2 (3)
    //     SW MMIO addr is in 64-bit space
    //     SW MMIO addr is 8B aligned (4)
    //     (3, 4) -> SW/HW = 4
    // =============================================================
    localparam NIC_MMIO_SW_ADDR = 32'h0000; // changed from h'20k to 0
    localparam NIC_MMIO_ADDR_SW_2_HW = NIC_MMIO_SW_ADDR/4;

    // Ethernet streams for nic
    logic eth_tx_clk;
    logic eth_tx_reset;
    logic eth_tx_ready [2:0];
    logic [255:0] eth_tx_data [2:0];
    logic eth_tx_valid [2:0];
    logic eth_tx_sop [2:0];
    logic eth_tx_eop [2:0];
    logic [4:0] eth_tx_empty [2:0];
    logic eth_tx_error [2:0];

    logic eth_rx_clk;
    logic eth_rx_reset;
    logic eth_rx_ready [2:0];
    logic [255:0] eth_rx_data [2:0];
    logic eth_rx_valid [2:0];
    logic eth_rx_sop [2:0];
    logic eth_rx_eop [2:0];
    logic [4:0] eth_rx_empty [2:0];
    logic [5:0] eth_rx_error [2:0];

    // eth signals
    logic eth_tx_ready_joint;
    logic [255:0] eth_tx_data_joint;
    logic eth_tx_valid_joint;
    logic eth_tx_sop_joint;
    logic eth_tx_eop_joint;
    logic [4:0] eth_tx_empty_joint;
    logic eth_tx_error_joint;

    logic eth_rx_ready_joint;
    logic [255:0] eth_rx_data_joint;
    logic eth_rx_valid_joint;
    logic eth_rx_sop_joint;
    logic eth_rx_eop_joint;
    logic [4:0] eth_rx_empty_joint;
    logic [5:0] eth_rx_error_joint;

    // NIC
    nic #(
            .NIC_ID(8'h00),
            .SRF_BASE_MMIO_ADDRESS(NIC_MMIO_ADDR_SW_2_HW),
            .SRF_BASE_MMIO_ADDRESS_AFU_ID(NIC_MMIO_ADDR_SW_2_HW),
            .NUM_SUB_AFUS(NUM_SUB_AFUS)
          ) nic_0 (
            .clk(clk),
            .clk_div_2(clk_div_2),
            .reset(ccip_mux2pe_reset[1]),

            .sRx(pck_afu_RxPort[1]),
            .sTx(pck_afu_TxPort[1]),

            .tx_clk_in (eth_tx_clk),
            .tx_reset_in (eth_tx_reset),
            .tx_ready_in (eth_tx_ready[0]),
            .tx_data_out (eth_tx_data[0]),
            .tx_valid_out (eth_tx_valid[0]),
            .tx_sop_out (eth_tx_sop[0]),
            .tx_eop_out (eth_tx_eop[0]),
            .tx_empty_out (eth_tx_empty[0]),
            .tx_error_out (eth_tx_error[0]),

            .rx_clk_in (eth_rx_clk),
            .rx_reset_in (eth_rx_reset),
            .rx_data_in (eth_rx_data[0]),
            .rx_valid_in (eth_rx_valid[0]),
            .rx_sop_in (eth_rx_sop[0]),
            .rx_eop_in (eth_rx_eop[0]),
            .rx_empty_in (eth_rx_empty[0]),
            .rx_error_in (eth_rx_error[0]),
            .rx_ready_out (eth_rx_ready[0])
        );

    // NIC
    nic #(
            .NIC_ID(8'h00),
            .SRF_BASE_MMIO_ADDRESS(NIC_MMIO_ADDR_SW_2_HW),
            .SRF_BASE_MMIO_ADDRESS_AFU_ID(NIC_MMIO_ADDR_SW_2_HW),
            .NUM_SUB_AFUS(NUM_SUB_AFUS)
          ) nic_1 (
            .clk(clk),
            .clk_div_2(clk_div_2),
            .reset(ccip_mux2pe_reset[1]),

            .sRx(pck_afu_RxPort[1]),
            .sTx(pck_afu_TxPort[1]),

            .tx_clk_in (eth_tx_clk),
            .tx_reset_in (eth_tx_reset),
            .tx_ready_in (eth_tx_ready[1]),
            .tx_data_out (eth_tx_data[1]),
            .tx_valid_out (eth_tx_valid[1]),
            .tx_sop_out (eth_tx_sop[1]),
            .tx_eop_out (eth_tx_eop[1]),
            .tx_empty_out (eth_tx_empty[1]),
            .tx_error_out (eth_tx_error[1]),

            .rx_clk_in (eth_rx_clk),
            .rx_reset_in (eth_rx_reset),
            .rx_data_in (eth_rx_data[1]),
            .rx_valid_in (eth_rx_valid[1]),
            .rx_sop_in (eth_rx_sop[1]),
            .rx_eop_in (eth_rx_eop[1]),
            .rx_empty_in (eth_rx_empty[1]),
            .rx_error_in (eth_rx_error[1]),
            .rx_ready_out (eth_rx_ready[1])
        );

    // NIC
    nic #(
            .NIC_ID(8'h00),
            .SRF_BASE_MMIO_ADDRESS(NIC_MMIO_ADDR_SW_2_HW),
            .SRF_BASE_MMIO_ADDRESS_AFU_ID(NIC_MMIO_ADDR_SW_2_HW),
            .NUM_SUB_AFUS(NUM_SUB_AFUS)
          ) nic_2 (
            .clk(clk),
            .clk_div_2(clk_div_2),
            .reset(ccip_mux2pe_reset[1]),

            .sRx(pck_afu_RxPort[1]),
            .sTx(pck_afu_TxPort[1]),

            .tx_clk_in (eth_tx_clk),
            .tx_reset_in (eth_tx_reset),
            .tx_ready_in (eth_tx_ready[2]),
            .tx_data_out (eth_tx_data[2]),
            .tx_valid_out (eth_tx_valid[2]),
            .tx_sop_out (eth_tx_sop[2]),
            .tx_eop_out (eth_tx_eop[2]),
            .tx_empty_out (eth_tx_empty[2]),
            .tx_error_out (eth_tx_error[2]),

            .rx_clk_in (eth_rx_clk),
            .rx_reset_in (eth_rx_reset),
            .rx_data_in (eth_rx_data[2]),
            .rx_valid_in (eth_rx_valid[2]),
            .rx_sop_in (eth_rx_sop[2]),
            .rx_eop_in (eth_rx_eop[2]),
            .rx_empty_in (eth_rx_empty[2]),
            .rx_error_in (eth_rx_error[2]),
            .rx_ready_out (eth_rx_ready[2])
        );

    t_if_ccip_Rx sRx_nic;
    assign sRx_nic = pck_afu_RxPort[1];

    t_ccip_c0_ReqMmioHdr mmio_req_hdr;
    assign mmio_req_hdr = t_ccip_c0_ReqMmioHdr'(sRx_nic.c0.hdr);

    // CSR write logic
    logic is_csr_write;
    assign is_csr_write = sRx_nic.c0.mmioWrValid;

    logic is_mem_tx_addr_csr_write;
    assign is_mem_tx_addr_csr_write = is_csr_write &&
                                      (mmio_req_hdr.address == addrRegMemTxAddr);

    logic is_mem_rx_addr_csr_write;
    assign is_mem_rx_addr_csr_write = is_csr_write &&
                                      (mmio_req_hdr.address == addrRegMemRxAddr);

    logic is_nic_start_csr_write;
    assign is_nic_start_csr_write = is_csr_write &&
                                    (mmio_req_hdr.address == addrRegNicStart);

    logic is_num_of_flows_csr_write;
    assign is_num_of_flows_csr_write = is_csr_write &&
                                       (mmio_req_hdr.address == addrRegNumOfFlows);

    logic is_init_csr_write;
    assign is_init_csr_write = is_csr_write &&
                               (mmio_req_hdr.address == addrRegInit);

    logic is_get_nic_cnt_csr_write;
    assign is_get_nic_cnt_csr_write = is_csr_write &&
                                      (mmio_req_hdr.address == addrGetPckCnt);

    logic is_rx_queue_size_csr_write;
    assign is_rx_queue_size_csr_write = is_csr_write &&
                                        (mmio_req_hdr.address == addrRxQueueSize);

    logic is_tx_queue_size_csr_write;
    assign is_tx_queue_size_csr_write = is_csr_write &&
                                        (mmio_req_hdr.address == addrTxQueueSize);

    logic is_tx_batch_size_csr_write;
    assign is_tx_batch_size_csr_write = is_csr_write &&
                                        (mmio_req_hdr.address == addrTxBatchSize);

    logic is_rx_batch_size_csr_write;
    assign is_rx_batch_size_csr_write = is_csr_write &&
                                        (mmio_req_hdr.address == addrRxBatchSize);

    logic is_polling_rate_csr_write;
    assign is_polling_rate_csr_write = is_csr_write &&
                                        (mmio_req_hdr.address == addrPollingRate);

    logic is_conn_setup_frame_write;
    assign is_conn_setup_frame_write = is_csr_write &&
                                        (mmio_req_hdr.address == addrConnSetup);

    logic is_lb_write;
    assign is_lb_write = is_csr_write &&
                                        (mmio_req_hdr.address == addrLB);

    logic is_phy_net_addr_write;
    assign is_phy_net_addr_write = is_csr_write &&
                                        (mmio_req_hdr.address == addrPhyNetAddr);

    logic is_ipv4_net_addr_write;
    assign is_ipv4_net_addr_write = is_csr_write &&
                                        (mmio_req_hdr.address == addrIPv4NetAddr);

    logic is_net_drop_cnt_read;
    assign is_net_drop_cnt_read = is_csr_write &&
                                        (mmio_req_hdr.address == addrNetDropCntRead);

    always_ff @(posedge ccip_clk) begin
        // Default values
        iRegNicInit <= 1'b0;
        iRegConnSetupFrame_en <= 1'b0;
        iRegReadNetDropCntValid <= 1'b0;

        if (is_mem_tx_addr_csr_write) begin
            $display("NIC%d: iRegMemTxAddr configured: %08h", NIC_ID, sRx_nic.c0.data);
            iRegMemTxAddr <= t_ccip_clAddr'(sRx_nic.c0.data);
        end

        if (is_mem_rx_addr_csr_write) begin
            $display("NIC%d: iRegMemRxAddr configured: %08h", NIC_ID, sRx_nic.c0.data);
            iRegMemRxAddr <= t_ccip_clAddr'(sRx_nic.c0.data);
        end

        if (is_nic_start_csr_write) begin
            $display("NIC%d: iRegNicStart configured: %08h", NIC_ID, sRx_nic.c0.data);
            iRegNicStart <= sRx_nic.c0.data[0];
        end

        if (is_num_of_flows_csr_write) begin
            $display("NIC%d: iRegNumOfFlows configured: %08h", NIC_ID, sRx_nic.c0.data);
            iRegNumOfFlows <= sRx_nic.c0.data[LMAX_NUM_OF_FLOWS-1:0] - 1;
        end

        if (is_init_csr_write) begin
            $display("NIC%d: iRegNicInit received", NIC_ID);
            iRegNicInit <= 1;
        end

        if (is_get_nic_cnt_csr_write) begin
            $display("NIC%d: iRegGetPckCnt received: %08h", NIC_ID, sRx_nic.c0.data);
            iRegGetPckCnt <= sRx_nic.c0.data[7:0];
        end

        if (is_rx_queue_size_csr_write) begin
            $display("NIC%d: iRegRxQueueSize received: %08h", NIC_ID, sRx_nic.c0.data);
            iRegRxQueueSize <= sRx_nic.c0.data[LMAX_RX_QUEUE_SIZE-1:0] - 1;
        end

        if (is_tx_queue_size_csr_write) begin
            $display("NIC%d: iRegTxQueueSize received: %08h", NIC_ID, sRx_nic.c0.data);
            iRegTxQueueSize <= sRx_nic.c0.data[LMAX_TX_QUEUE_SIZE:0];
        end

        if (is_tx_batch_size_csr_write) begin
            $display("NIC%d: lRegTxBatchSize received: %08h", NIC_ID, sRx_nic.c0.data);
            lRegTxBatchSize <= sRx_nic.c0.data[LMAX_CCIP_BATCH-1:0];
        end

        if (is_rx_batch_size_csr_write) begin
            $display("NIC%d: iRegRxBatchSize received: %08h", NIC_ID, sRx_nic.c0.data);
            iRegRxBatchSize <= sRx_nic.c0.data[LMAX_CCIP_DMA_BATCH-1:0];
        end

        if (is_polling_rate_csr_write) begin
            $display("NIC%d: iRegPollingRate received: %08h", NIC_ID, sRx_nic.c0.data);
            iRegPollingRate <= sRx_nic.c0.data[LMAX_POLLING_RATE-1:0];
        end

        if (is_conn_setup_frame_write) begin
            $display("NIC%d: iRegConnSetupFrame received: %08h", NIC_ID, sRx_nic.c0.data);
            iRegConnSetupFrame <= sRx_nic.c0.data[$bits(ConnSetupFrame)-1:0];
            iRegConnSetupFrame_en <= 1'b1;
        end

        if (is_lb_write) begin
            iLB <= sRx_nic.c0.data[$bits(iLB)-1:0];
        end

        if (is_phy_net_addr_write) begin
            iRegPhyNetAddr.b0 <= sRx_nic.c0.data[7:0];
            iRegPhyNetAddr.b1 <= sRx_nic.c0.data[15:8];
            iRegPhyNetAddr.b2 <= sRx_nic.c0.data[23:16];
            iRegPhyNetAddr.b3 <= sRx_nic.c0.data[31:24];
            iRegPhyNetAddr.b4 <= sRx_nic.c0.data[39:32];
            iRegPhyNetAddr.b5 <= sRx_nic.c0.data[47:40];
        end

        if (is_ipv4_net_addr_write) begin
            iRegIpv4NetAddr.b0 <= sRx_nic.c0.data[7:0];
            iRegIpv4NetAddr.b1 <= sRx_nic.c0.data[15:8];
            iRegIpv4NetAddr.b2 <= sRx_nic.c0.data[23:16];
            iRegIpv4NetAddr.b3 <= sRx_nic.c0.data[31:24];
        end

        if (is_net_drop_cnt_read) begin
            iRegReadNetDropCnt <= sRx_nic.c0.data[3:0];
            iRegReadNetDropCntValid <= 1'b1;
        end

        if (reset) begin
            iRegNicStart <= 1'b0;
            iRegNicInit  <= 1'b0;
            iRegConnSetupFrame_en <= 1'b0;
            iRegReadNetDropCntValid <= 1'b0;
        end
    end

    // udp wires 

    PhyAddr host_phy_addr_udp;
    IPv4 host_ipv4_addr_udp;

    // App interface
    logic reset_udp;
    logic clk_udp;
    NetworkIf network_tx_in_udp;
    // logic [15:0] network_tx_size_in_udp;
    NetworkIf network_rx_out_udp;

    // Networking MAC/PHY interface
    // TX Avalon-ST interface
    logic tx_clk_in_udp;
    logic tx_reset_in_udp;
    logic tx_ready_in_udp;
    logic [255:0] tx_data_out_udp;
    logic         tx_valid_out_udp;
    logic         tx_sop_out_udp;
    logic         tx_eop_out_udp;
    logic [4:0]   tx_empty_out_udp;
    logic         tx_error_out_udp;

    // RX Avalon-ST interface
    logic rx_clk_in_udp;
    logic rx_reset_in_udp;
    logic [255:0]   rx_data_in_udp;
    logic rx_valid_in_udp;
    logic rx_sop_in_udp;
    logic rx_eop_in_udp;
    logic [4:0]   rx_empty_in_udp;
    logic [5:0]   rx_error_in_udp;
    logic         rx_ready_out_udp;

    // Drop counter interfaces
    logic [3:0]   pckt_drop_cnt_in_udp;
    logic pckt_drop_cnt_valid_in_udp;
    logic [63:0] pckt_drop_cnt_out_udp;

    // Error
    // logic error_udp



    udp_ip udp_ (
            .host_phy_addr(host_phy_addr_udp),
            .host_ipv4_addr(host_ipv4_addr_udp),

            .clk(clk_udp), // should we have one universal clock
            .reset(reset_udp),
            .network_tx_in(network_tx_in_udp),
            .network_tx_size_in(16'd64), // must be size of data payload only 
                                         // not including addr_tpl or valid
                                         // or remote_qp_num etc.
            .network_rx_out(network_rx_out_udp),

            .tx_clk_in (tx_clk_in_udp),
            .tx_reset_in (tx_reset_in_udp),
            .tx_ready_in (tx_ready_in_udp),
            .tx_data_out (tx_data_out_udp),
            .tx_valid_out (tx_valid_out_udp),
            .tx_sop_out (tx_sop_out_udp),
            .tx_eop_out (tx_eop_out_udp),
            .tx_empty_out (tx_empty_out_udp),
            .tx_error_out (tx_error_out_udp),

            .rx_clk_in (rx_clk_in_udp),
            .rx_reset_in (rx_reset_in_udp),
            .rx_data_in (rx_data_in_udp),
            .rx_valid_in (rx_valid_in_udp),
            .rx_sop_in (rx_sop_in_udp),
            .rx_eop_in (rx_eop_in_udp),
            .rx_empty_in (rx_empty_in_udp),
            .rx_error_in (rx_error_in_udp),
            .rx_ready_out (rx_ready_out_udp),

            .pckt_drop_cnt_in(pckt_drop_cnt_in_udp),
            .pckt_drop_cnt_valid_in(pckt_drop_cnt_valid_in_udp),
            .pckt_drop_cnt_out(pckt_drop_cnt_out_udp),

            .error()
        );


    always @(posedge network_clk) begin
        // network_Rx_line   <= network_Tx_line_1;
        // network_Rx_line_1 <= network_Tx_line;

        // if (network_rst) begin
        //     network_Rx_line.valid <= 1'b0;
        //     network_Rx_line_1.valid <= 1'b0;
        // end

        eth_rx_valid[0] <= 1'b0;
        eth_rx_valid[1] <= 1'b0;
        eth_rx_valid[2] <= 1'b0;
        eth_rx_valid[3] <= 1'b0;

        if (eth_tx_valid[0] == 1'b1) begin
            
        end
        if (network_Tx_line[1].valid == 1'b1) begin
            network_Rx_line[network_Tx_line[1].addr_tpl.dest_ip.b0] <= network_Tx_line[1];
        end
        if (network_Tx_line[2].valid == 1'b1) begin
            network_Rx_line[network_Tx_line[2].addr_tpl.dest_ip.b0] <= network_Tx_line[2];
        end
        if (network_Tx_line[3].valid == 1'b1) begin
            network_Rx_line[network_Tx_line[3].addr_tpl.dest_ip.b0] <= network_Tx_line[3];
        end

        // LOOK AT THESE
        host_phy_addr_udp <= iRegPhyNetAddr;
        host_ipv4_addr_udp <= iRegIpv4NetAddr;

        reset_udp <= network_rst; // guessing
        clk_udp <= clk;
        
        network_tx_in_udp

        tx_clk_in_udp

        tx_reset_in_udp <= tx_reset_in[i];
        tx_ready_in_udp <= tx_ready_in[i];

        rx_clk_in_udp <= rx_clk_in[i];
        rx_reset_in_udp <= rx_reset_in[i];
        rx_data_in_udp <= rx_data_in[i];
        rx_valid_in_udp <= rx_data_in[i];
        rx_sop_in_udp <= rx_sop_in[i];
        rx_eop_in_udp <= rx_eop_in[i];
        rx_empty_in_udp <= rx_empty_in[i];
        rx_error_in_udp <= rx_error_in[i];

        pckt_drop_cnt_in_udp
        pckt_drop_cnt_valid_in_udp

        // OUTPUTS
        network_rx_out_udp,

        tx_data_out_udp,
        tx_valid_out_udp,
        tx_sop_out_udp,
        tx_eop_out_udp,
        tx_empty_out_udp,
        tx_error_out_udp,
        
        rx_ready_out_udp,

        pckt_drop_cnt_out_udp,

        if (network_rst) begin
            network_Rx_line[0].valid <= 1'b0;
            network_Rx_line[1].valid <= 1'b0;
            network_Rx_line[2].valid <= 1'b0;
            network_Rx_line[3].valid <= 1'b0;
        end
    end

    // =============================================================
    // Instantiate an Ethernet MAC/PHY
    //   - as CCI-P device #0, so we reserve the lower CSR addresses to control it
    //     via the standard SW library
    // =============================================================
    ethernet_mac eth_ (
            .clk(clk),
            .reset(ccip_mux2pe_reset[0]),

            .sRx(pck_afu_RxPort[0]),
            .sTx(pck_afu_TxPort[0]),

            .tx_clk_out   (eth_tx_clk),
            .tx_reset_out (eth_tx_reset),
            .tx_ready_out (eth_tx_ready),
            .tx_data_in  (eth_tx_data),
            .tx_valid_in (eth_tx_valid),
            .tx_sop_in   (eth_tx_sop),
            .tx_eop_in   (eth_tx_eop),
            .tx_empty_in (eth_tx_empty),
            .tx_error_in (eth_tx_error),

            .rx_clk_out   (eth_rx_clk),
            .rx_reset_out (eth_rx_reset),
            .rx_data_out  (eth_rx_data),
            .rx_valid_out (eth_rx_valid),
            .rx_sop_out   (eth_rx_sop),
            .rx_eop_out   (eth_rx_eop),
            .rx_empty_out (eth_rx_empty),
            .rx_error_out (eth_rx_error),
            .rx_ready_in (eth_rx_ready),

            .hssi(hssi)
        );

endmodule
