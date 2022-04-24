// Author: Cornell University
//
// Module Name :    top_level_network_module_3
// Project :        F-NIC
// Description :    Top-level module for the design
//                    - using real networking
//                    - instantiates one NIC and the networking MAC/PHY
//

`include "platform_if.vh"

`include "nic.sv"

//`include "nic_defs.vh"

module top_level_network_module_3 (
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
            .reset(ccip_mux2pe_reset[0]),

            .sRx(pck_afu_RxPort[0]),
            .sTx(pck_afu_TxPort[0]),

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
            .reset(ccip_mux2pe_reset[2]),

            .sRx(pck_afu_RxPort[2]),
            .sTx(pck_afu_TxPort[2]),

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

    
    logic [1:0] count;
    logic [1:0] count_next;
    logic [2:0] state;
    logic [2:0] next_state;

    localparam S_NIC1 = 2'd0;
    localparam S_NIC2 = 2'd1;
    localparam S_NIC3 = 2'd2;
    localparam S_IDLE = 2'd3;

    localparam S_COUNT0 = 2'd0;
    localparam S_COUNT1 = 2'd1;
    localparam S_COUNT2 = 2'd2;

    // ------------------------------
    // NIC FSM
    // ------------------------------

    // output logic

    always@ (posedge clk) begin

        eth_tx_ready[0] <= 1'b0;
        eth_tx_ready[1] <= 1'b0;
        eth_tx_ready[2] <= 1'b0;

        case (state)
            S_IDLE: begin
                eth_tx_data_joint <= eth_tx_data[0];
                eth_tx_valid_joint <= 1'b0;
                eth_tx_sop_joint <= eth_tx_sop[0];
                eth_tx_eop_joint <= eth_tx_eop[0];
                eth_tx_empty_joint <= eth_tx_empty[0];
                eth_tx_error_joint <= eth_tx_error[0];

                eth_rx_ready_joint <= eth_rx_ready[0];

            end
            
            default: begin

                eth_tx_data_joint <= eth_tx_data[state];
                eth_tx_valid_joint <= eth_tx_valid[state];
                eth_tx_sop_joint <= eth_tx_sop[state];
                eth_tx_eop_joint <= eth_tx_eop[state];
                eth_tx_empty_joint <= eth_tx_empty[state];
                eth_tx_error_joint <= eth_tx_error[state];

                eth_rx_ready_joint <= eth_rx_ready[state];

                eth_tx_ready[state] <= 1'b1;
            end
        endcase
    end

    // next stage logic
    always@ (posedge clk) begin
        case (state)
            S_IDLE: begin 
                if (count == 0 && eth_tx_valid[0] && eth_tx_ready[0]) next_state = S_NIC1;
                else if (count == 0 && eth_tx_valid[1] && eth_tx_ready[1]) next_state = S_NIC2;
                else if (count == 0 && eth_tx_valid[2] && eth_tx_ready[2]) next_state = S_NIC3;
                else next_state = S_IDLE;
            end

            S_NIC1: begin 
                if ((count == 1 || count == 2) && eth_tx_valid[0] && eth_tx_ready[0]) next_state = S_NIC1;
                else next_state = S_IDLE;
            end
            
            S_NIC2: begin 
                if ((count == 1 || count == 2) && eth_tx_valid[1] && eth_tx_ready[1]) next_state = S_NIC2;
                else next_state = S_IDLE;
            end

            S_NIC3: begin 
                if ((count == 1 || count == 2)  && eth_tx_valid[2] && eth_tx_ready[2]) next_state = S_NIC3;
                else next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // ------------------------------
    // COUNT FSM 
    // ------------------------------ 

    // next stage logic
    always@ (posedge clk) begin
        case (count)
            S_COUNT0: begin
                if ((eth_tx_valid[0] && eth_tx_ready[0]) || (eth_tx_valid[1] && eth_tx_ready[1]) 
                || (eth_tx_valid[2] && eth_tx_ready[2]))

                count_next = S_COUNT1;
            end

            S_COUNT1: count_next = S_COUNT2;
            S_COUNT2: count_next = S_COUNT0;
            
            default: count_next = S_COUNT0;
            
        endcase
    end

    // state transition logic
    always@(posedge clk) begin
    if (ccip_mux2pe_reset[1]) 
        count <= S_COUNT0;
    else
        count <= count_next;
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
            .tx_ready_out (eth_tx_ready_joint),
            .tx_data_in  (eth_tx_data_joint),
            .tx_valid_in (eth_tx_valid_joint),
            .tx_sop_in   (eth_tx_sop_joint),
            .tx_eop_in   (eth_tx_eop_joint),
            .tx_empty_in (eth_tx_empty_joint),
            .tx_error_in (eth_tx_error_joint),

            .rx_clk_out   (eth_rx_clk),
            .rx_reset_out (eth_rx_reset),
            .rx_data_out  (eth_rx_data_joint),
            .rx_valid_out (eth_rx_valid_joint),
            .rx_sop_out   (eth_rx_sop_joint),
            .rx_eop_out   (eth_rx_eop_joint),
            .rx_empty_out (eth_rx_empty_joint),
            .rx_error_out (eth_rx_error_joint),
            .rx_ready_in (eth_rx_ready_joint),

            .hssi(hssi)
        );

endmodule
