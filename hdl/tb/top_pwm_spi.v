// top_pwm_spi.v
//
// A synthesis wrapper: it instantiates the SPI and PWM modules side by side so
// yosys can report what the combination costs on an iCE40HX1K. It contains no
// controller and no glue logic, so its LUT count is a FLOOR for an integrated
// SPI + PI + PWM top level, not a model of one.

module top_pwm_spi (
    input        clk,
    input        rst,
    // servo / ESC PWM
    input        increase_duty0,
    input        decrease_duty0,
    input        increase_duty1,
    input        decrease_duty1,
    output       PWM_OUT0,
    output       PWM_OUT1,
    // SPI master
    input        miso,
    output       mosi,
    output       sck,
    input        start,
    input  [7:0] data_in,
    output [7:0] data_out,
    output       busy,
    output       new_data
);

  PWM_Generator_Verilog u_pwm (
    .clk(clk),
    .increase_duty0(increase_duty0), .decrease_duty0(decrease_duty0),
    .increase_duty1(increase_duty1), .decrease_duty1(decrease_duty1),
    .PWM_OUT0(PWM_OUT0), .PWM_OUT1(PWM_OUT1)
  );

  spi #(.CLK_DIV(4)) u_spi (
    .clk(clk), .rst(rst),
    .miso(miso), .mosi(mosi), .sck(sck),
    .start(start), .data_in(data_in), .data_out(data_out),
    .busy(busy), .new_data(new_data)
  );

endmodule
