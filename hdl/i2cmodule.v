// i2cmodule.v
//
// A thin wrapper around `i2c_master_byte_ctrl`, the byte controller from the
// OpenCores I2C master core. That core is third-party and is not included
// here; tb/i2c_master_byte_ctrl_stub.v carries a black-box declaration of it so
// this wrapper elaborates.
//
// STATUS: this compiles and elaborates. It is NOT working I2C. The transaction
// sequencing (start / address / R-W bit / ack / stop) lives inside the
// OpenCores byte controller, which is absent, and this wrapper does not
// sequence it. Do not present this as implemented I2C firmware.

`timescale 1ns / 1ps

module i2c_master_single_byte #(parameter CLK_RATIO = 25)
    (
       input        i_clk,
       input        i_rst,
       input        i_enable,

       input  [6:0] i_slave_addr,
       input        i_write_start,
       input        i_rd_start,
       input  [7:0] i_wrbyte,
       output reg   o_busy,
       output [7:0] o_rd_byte,
       output       o_error,
       output       o_done,

       inout        io_scl,
       inout        io_sda
   );

    localparam [15:0] CLK_COUNT = CLK_RATIO / 5 - 1;

    wire w_sck_en, w_sda_en;
    wire w_arb_lost, w_cmd_ack, w_slave_ack;
    wire w_cmd_start;
    reg  r_cmd_stop;

    assign o_done  = w_arb_lost | w_cmd_ack;
    // arbitration loss, or the slave failing to acknowledge, is an error
    assign o_error = w_arb_lost | (w_cmd_ack & w_slave_ack);

    // command start is either a read or a write, never both -- hence the XOR
    assign w_cmd_start = i_write_start ^ i_rd_start;

    // issue a stop one cycle after the byte controller acknowledges
    always @(posedge i_clk) begin
        if (i_rst) r_cmd_stop <= 1'b0;
        else       r_cmd_stop <= w_cmd_ack;
    end

    always @(posedge i_clk) begin
        if (i_rst)                o_busy <= 1'b0;
        else if (w_cmd_start)     o_busy <= 1'b1;
        else if (w_cmd_ack)       o_busy <= 1'b0;
    end

    // byte controller from the OpenCores I2C master core -- not included here
    i2c_master_byte_ctrl byte_controller (
        .clk     (i_clk        ),
        .rst     (1'b0         ),
        .nReset  (~i_rst       ),
        .ena     (i_enable     ),
        .clk_cnt (CLK_COUNT    ),
        .start   (w_cmd_start  ),
        .stop    (r_cmd_stop   ),
        .read    (i_rd_start   ),
        .write   (i_write_start),
        .ack_in  (1'b0         ),
        .din     (i_wrbyte     ),
        .cmd_ack (w_cmd_ack    ),
        .ack_out (w_slave_ack  ),
        .dout    (o_rd_byte    ),
        .i2c_busy(             ),
        .i2c_al  (w_arb_lost   ),
        .scl_i   (io_scl       ),
        .scl_o   (             ),
        .scl_oen (w_sck_en     ),
        .sda_i   (io_sda       ),
        .sda_o   (             ),
        .sda_oen (w_sda_en     )
    );

    // open-drain buffers: enable high releases the line, enable low pulls it down
    assign io_scl = w_sck_en ? 1'bZ : 1'b0;
    assign io_sda = w_sda_en ? 1'bZ : 1'b0;

endmodule
