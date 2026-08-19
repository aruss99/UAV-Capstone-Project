// hdlcoder_stubs.v
//
// HDLDroneModel3.v instantiates 8 modules whose definitions are not present.
// These are empty black boxes with matching port lists, taken from the
// named-port connections at the instantiation sites. They let yosys
// resolve the hierarchy and report the cost of the TOP-LEVEL GLUE ONLY:
// the gain-schedule lookup tables, prelookups, muxing and pipeline registers.
//
// Everything expensive is inside these boxes -- the three PI controllers and
// every single-precision floating-point operator -- so any LUT count produced
// this way is a hard FLOOR, not an estimate of the real design.

(* blackbox *) module nfp_wire_single (input [31:0] nfp_in, output [31:0] nfp_out);
endmodule

(* blackbox *) module nfp_mul_single (
  input clk, input reset, input enb,
  input [31:0] nfp_in1, input [31:0] nfp_in2, output [31:0] nfp_out);
endmodule

(* blackbox *) module nfp_sub_single (
  input clk, input reset, input enb,
  input [31:0] nfp_in1, input [31:0] nfp_in2, output [31:0] nfp_out);
endmodule

(* blackbox *) module nfp_gain_pow2_single (
  input clk, input reset, input enb,
  input [31:0] nfp_in1, input nfp_in2, input [8:0] nfp_in3,
  output [31:0] nfp_out);
endmodule

// The enable named enb_1_20000000_1 is HDL Coder's "assert once every
// 20,000,000 clock cycles" strobe. It is what gates the three PI controllers.
(* blackbox *) module HDLDroneModel3_tc (
  input clk, input reset, input clk_enable,
  output enb, output enb_1_1_1, output enb_1_20000000_1);
endmodule

(* blackbox *) module Discrete_PID_Controller (
  input clk, input reset, input enb_1_20000000_1, input enb,
  input [31:0] u, input [31:0] P, input [31:0] I, output [31:0] y);
endmodule

(* blackbox *) module Discrete_PID_Controller1 (
  input clk, input reset, input enb_1_20000000_1, input enb,
  input [31:0] u, input [31:0] P, input [31:0] I, output [31:0] y);
endmodule

(* blackbox *) module Discrete_PID_Controller2 (
  input clk, input reset, input enb_1_20000000_1, input enb,
  input [31:0] u, input [31:0] P, input [31:0] I, output [31:0] y);
endmodule
