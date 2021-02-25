// altera_rand_gen.v

// Generated using ACDS version 16.0 211

`include "altera_rand_gen_internal.v"

`timescale 1 ps / 1 ps
module altera_rand_gen (
		input  wire        start,          //     call.valid
		output wire        busy,           //         .stall
		input  wire        clock,          //    clock.clk
		output wire [31:0] rand_num_data,  // rand_num.data
		input  wire        rand_num_ready, //         .ready
		output wire        rand_num_valid, //         .valid
		input  wire        resetn,         //    reset.reset_n
		output wire        done,           //   return.valid
		input  wire        stall           //         .stall
	);

	altera_rand_gen_internal altera_rand_gen_internal_inst (
		.clock          (clock),          //    clock.clk
		.resetn         (resetn),         //    reset.reset_n
		.rand_num_data  (rand_num_data),  // rand_num.data
		.rand_num_ready (rand_num_ready), //         .ready
		.rand_num_valid (rand_num_valid), //         .valid
		.start          (start),          //     call.valid
		.busy           (busy),           //         .stall
		.done           (done),           //   return.valid
		.stall          (stall)           //         .stall
	);

endmodule