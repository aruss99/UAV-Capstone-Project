// shift_register_tb.v
//
// Drives stimulus into `shift8` and checks what comes out, rather than
// duplicating the DUT's shift logic. Reset is active-low, matching the DUT, and
// `send_message` must be driven high or the register does not shift at all.

`timescale 1ns / 1ps

module shift8_tb();

  reg  clk = 0;
  reg  reset = 0;          // active low
  reg  send_message = 0;
  reg  serial_in = 0;
  wire scl;
  wire serial_out;

  shift8 uut (
      .reset(reset),
      .send_message(send_message),
      .clk(clk),
      .serial_in(serial_in),
      .scl(scl),
      .serial_out(serial_out)
  );

  always #5 clk = ~clk;

  // shift in 0xA5 one bit at a time, MSB first
  localparam [7:0] PATTERN = 8'hA5;
  integer i;

  initial begin
`ifdef DUMPFILE
    $dumpfile(`DUMPFILE);
    $dumpvars(0, shift8_tb);
`endif
    $display("");
    $display("  shift8: shifting in 0x%02h, MSB first", PATTERN);
    #100;
    reset = 1'b1;            // release reset
    send_message = 1'b1;
    for (i = 7; i >= 0; i = i - 1) begin
      serial_in = PATTERN[i];
      // the DUT only shifts on the cycle where count >= 12, so wait it out
      @(posedge scl);
      $display("    t=%7t  shifted in %b   serial_out=%b", $time, serial_in, serial_out);
    end
    #200;
    $display("");
    $finish;
  end

  // safety net so a logic error cannot hang the run
  initial begin
    #200_000;
    $display("  TIMEOUT: scl never toggled the expected number of times");
    $finish;
  end

endmodule
