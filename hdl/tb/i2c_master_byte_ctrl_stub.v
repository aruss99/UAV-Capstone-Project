// i2c_master_byte_ctrl_stub.v
// Not an implementation.
//
// i2cmodule.v wraps `i2c_master_byte_ctrl`, the byte controller from the
// OpenCores I2C master core (Richard Herveille, i2c_master_byte_ctrl.v). That
// core is third-party and is not included here.
//
// This black box carries the port list the wrapper connects to, so the wrapper
// can elaborate and be checked. It implements nothing. Substituting the real
// OpenCores core here is what would be needed to make the I2C path functional.

(* blackbox *)
module i2c_master_byte_ctrl (
    input        clk,
    input        rst,
    input        nReset,
    input        ena,
    input [15:0] clk_cnt,
    input        start,
    input        stop,
    input        read,
    input        write,
    input        ack_in,
    input  [7:0] din,
    output       cmd_ack,
    output       ack_out,
    output [7:0] dout,
    output       i2c_busy,
    output       i2c_al,
    input        scl_i,
    output       scl_o,
    output       scl_oen,
    input        sda_i,
    output       sda_o,
    output       sda_oen
);
endmodule
