// e40_tx_pll_644.v

// Generated using ACDS version 16.0.2 222

`timescale 1 ps / 1 ps
module e40_tx_pll_644 (
		input  wire  rst,      //   reset.reset
		input  wire  refclk,   //  refclk.clk
		output wire  locked,   //  locked.export
		output wire  outclk_0, // outclk0.clk
		output wire  outclk_1  // outclk1.clk
	);

	e40_altera_iopll_160_h3enegy e40_tx_pll_644 (
		.rst      (rst),      //   reset.reset
		.refclk   (refclk),   //  refclk.clk
		.locked   (locked),   //  locked.export
		.outclk_0 (outclk_0), // outclk0.clk
		.outclk_1 (outclk_1)  // outclk1.clk
	);

endmodule