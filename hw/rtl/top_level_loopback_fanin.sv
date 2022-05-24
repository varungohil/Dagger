// Author: Cornell University
//
// Module Name :    top_level_loopback_fanin_module
// Project :        F-NIC
// Description :    Top-level module for the design
//                    - using loopback
//                    - instantiates four NICs
//                      (3 nodes fanning into 1 node)
//

`include "platform_if.vh"

`include "nic.sv"

//`include "nic_defs.vh"

module top_level_loopback_fanin_module (
    // CCI-P Clocks and Resets
    input           logic             pClk,          // 200MHz
    input           logic             pClkDiv2,      // 100MHz
    input           logic             pReset,

    // Host interface
    input           t_if_ccip_Rx      pck_cp2af_sRx,       // CCI-P Rx Port
    output          t_if_ccip_Tx      pck_af2cp_sTx        // CCI-P Tx Port
    );

    // We need four NICs:
    // * 3 client-side NICs
    // * 1 server-side NIC
    localparam NUM_SUB_AFUS    = 4; // # of nics on one server?
    localparam NUM_PIPE_STAGES = 2; // TODO: figure out what this does


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
    // Install NIC devices
    // =============================================================
    NetworkIf network_Tx_line[NUM_SUB_AFUS];
    NetworkIf network_Rx_line[NUM_SUB_AFUS];

    // NIC_0:
    //     TODO: So far, NIC::connect_to_accel always reads UUID by 0x00000
    //     so it always reads from nic_0
    //     fix it in software: different NICs must have different MMIO space

    // logic nic_0_reset;
    // t_if_ccip_Rx nic_0_rx;
    // t_if_ccip_Tx nic_0_tx;

    nic #(
            .NIC_ID(8'h00),
            .SRF_BASE_MMIO_ADDRESS(32'h00000),
            .SRF_BASE_MMIO_ADDRESS_AFU_ID(32'h00000),
            .NUM_SUB_AFUS(NUM_SUB_AFUS)
          ) nic_0 (
            .clk(clk),
            .clk_div_2(clk_div_2),
            .reset(ccip_mux2pe_reset[0]),

            .sRx(pck_afu_RxPort[0]),
            .sTx(pck_afu_TxPort[0]),

            .network_tx_out(network_Tx_line[0]),
            .network_rx_in(network_Rx_line[0])
        );

    // NIC_1:
    //     HW MMIO addr is in 32-bit space (1)
    //     HW MMIO objects are 64-bits (2)
    //     (1, 2) -> HW MMIO addresses are mult. of 2 (3)
    //     SW MMIO addr is in 64-bit space
    //     SW MMIO addr is 8B aligned (4)
    //     (3, 4) -> SW/HW = 4
    localparam NIC_1_MMIO_SW_ADDR = 32'h4000; // was 2 before? changed to match top_level.sv from u_services branch
    localparam NIC_1_MMIO_ADDR_SW_2_HW = NIC_1_MMIO_SW_ADDR/4;

    // logic nic_1_reset;
    // t_if_ccip_Rx nic_1_rx;
    // t_if_ccip_Tx nic_1_tx;

    nic #(
            .NIC_ID(8'h01),
            .SRF_BASE_MMIO_ADDRESS(NIC_1_MMIO_ADDR_SW_2_HW),
            .SRF_BASE_MMIO_ADDRESS_AFU_ID(NIC_1_MMIO_ADDR_SW_2_HW),
            .NUM_SUB_AFUS(NUM_SUB_AFUS)
          ) nic_1 (
            .clk(clk),
            .clk_div_2(clk_div_2),
            .reset(ccip_mux2pe_reset[1]),

            .sRx(pck_afu_RxPort[1]),
            .sTx(pck_afu_TxPort[1]),

            .network_tx_out(network_Tx_line[1]),
            .network_rx_in(network_Rx_line[1])
        );

    // NIC_2:
    //     HW MMIO addr is in 32-bit space (1)
    //     HW MMIO objects are 64-bits (2)
    //     (1, 2) -> HW MMIO addresses are mult. of 2 (3)
    //     SW MMIO addr is in 64-bit space
    //     SW MMIO addr is 8B aligned (4)
    //     (3, 4) -> SW/HW = 4
    localparam NIC_2_MMIO_SW_ADDR = 32'h8000;
    localparam NIC_2_MMIO_ADDR_SW_2_HW = NIC_2_MMIO_SW_ADDR/4;  // 2000

    nic #(
            .NIC_ID(8'h02),
            .SRF_BASE_MMIO_ADDRESS(NIC_2_MMIO_ADDR_SW_2_HW),
            .SRF_BASE_MMIO_ADDRESS_AFU_ID(NIC_2_MMIO_ADDR_SW_2_HW),
            .NUM_SUB_AFUS(NUM_SUB_AFUS)
          ) nic_2 (
            .clk(clk),
            .clk_div_2(clk_div_2),
            .reset(ccip_mux2pe_reset[2]),

            .sRx(pck_afu_RxPort[2]),
            .sTx(pck_afu_TxPort[2]),

            .network_tx_out(network_Tx_line[2]),
            .network_rx_in(network_Rx_line[2])
        );


    // NIC_3:
    //     HW MMIO addr is in 32-bit space (1)
    //     HW MMIO objects are 64-bits (2)
    //     (1, 2) -> HW MMIO addresses are mult. of 2 (3)
    //     SW MMIO addr is in 64-bit space
    //     SW MMIO addr is 8B aligned (4)
    //     (3, 4) -> SW/HW = 4
    localparam NIC_3_MMIO_SW_ADDR = 32'hC000;
    localparam NIC_3_MMIO_ADDR_SW_2_HW = NIC_3_MMIO_SW_ADDR/4;  // 3000

    nic #(
            .NIC_ID(8'h03),
            .SRF_BASE_MMIO_ADDRESS(NIC_3_MMIO_ADDR_SW_2_HW),
            .SRF_BASE_MMIO_ADDRESS_AFU_ID(NIC_3_MMIO_ADDR_SW_2_HW),
            .NUM_SUB_AFUS(NUM_SUB_AFUS)
          ) nic_3 (
            .clk(clk),
            .clk_div_2(clk_div_2),
            .reset(ccip_mux2pe_reset[3]),

            .sRx(pck_afu_RxPort[3]),
            .sTx(pck_afu_TxPort[3]),

            .network_tx_out(network_Tx_line[3]),
            .network_rx_in(network_Rx_line[3])
        );

    // =============================================================
    // Emulate ToR network as a loop-back connection with latency
    //   - current latency = 1 cycle
    //   - also do packet switching at L3
    // =============================================================
    logic network_clk;
    assign network_clk = clk_div_2;

    logic network_rst;
    assign network_rst = ccip_mux2pe_reset[0];

    // Problem with the logic in the u_services branch:
    //     seems like only 1 node can transmit at a time?
    //     nevermind, has if statements, no else
    //     we'll have to modify to pick queue pair within node?

    always @(posedge network_clk) begin
        // network_Rx_line   <= network_Tx_line_1;
        // network_Rx_line_1 <= network_Tx_line;

        // if (network_rst) begin
        //     network_Rx_line.valid <= 1'b0;
        //     network_Rx_line_1.valid <= 1'b0;
        // end

        network_Rx_line[0].valid <= 1'b0;
        network_Rx_line[1].valid <= 1'b0;
        network_Rx_line[2].valid <= 1'b0;
        network_Rx_line[3].valid <= 1'b0;

        if (network_Tx_line[0].valid == 1'b1) begin
            network_Rx_line[network_Tx_line[0].addr_tpl.dest_ip.b0] <= network_Tx_line[0];
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

        if (network_rst) begin
            network_Rx_line[0].valid <= 1'b0;
            network_Rx_line[1].valid <= 1'b0;
            network_Rx_line[2].valid <= 1'b0;
            network_Rx_line[3].valid <= 1'b0;
        end
    end


endmodule
