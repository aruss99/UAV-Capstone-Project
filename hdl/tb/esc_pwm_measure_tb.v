// esc_pwm_measure_tb.v
//
// Holds the buttons idle, clocks at a true 12 MHz, and measures the period and
// high-time of both PWM outputs directly. CODE_WORKS/ESC_PWM_tb.v drives the
// buttons but measures nothing, and clocks at 100 MHz (#5 half-period on a 1 ns
// timescale) rather than the 12 MHz the design assumes.
//
// Compile with -DDUMPFILE=... to also emit a VCD.

`timescale 1ns / 1ps

module esc_pwm_measure_tb();

  // 12 MHz -> 83.3333 ns period -> 41.6667 ns half period
  localparam real HALF_NS = 41.66667;

  reg clk = 0;
  reg increase_duty0 = 0, decrease_duty0 = 0;
  reg increase_duty1 = 0, decrease_duty1 = 0;
  wire PWM_OUT0, PWM_OUT1;

  PWM_Generator_Verilog uut (
    .clk(clk),
    .increase_duty0(increase_duty0), .decrease_duty0(decrease_duty0),
    .increase_duty1(increase_duty1), .decrease_duty1(decrease_duty1),
    .PWM_OUT0(PWM_OUT0), .PWM_OUT1(PWM_OUT1)
  );

  always #HALF_NS clk = ~clk;

  // ---- measurement ----------------------------------------------------
  real t_rise0, t_fall0, t_rise0_prev;
  real t_rise1, t_fall1, t_rise1_prev;
  integer n0 = 0, n1 = 0;

  task report(input [8*8-1:0] tag, input real period_ns, input real high_ns);
    begin
      $display("  %0s : period = %0.3f us (%0.2f Hz)   high = %0.3f us   duty = %0.2f %%",
               tag, period_ns/1000.0, 1.0e9/period_ns, high_ns/1000.0,
               100.0*high_ns/period_ns);
    end
  endtask

  always @(posedge PWM_OUT0) begin
    t_rise0_prev = t_rise0;
    t_rise0 = $realtime;
    n0 = n0 + 1;
    if (n0 >= 2) report("SERVO ", t_rise0 - t_rise0_prev, t_fall0 - t_rise0_prev);
  end
  always @(negedge PWM_OUT0) t_fall0 = $realtime;

  always @(posedge PWM_OUT1) begin
    t_rise1_prev = t_rise1;
    t_rise1 = $realtime;
    n1 = n1 + 1;
    if (n1 >= 2 && n1 <= 4) report("ESC   ", t_rise1 - t_rise1_prev, t_fall1 - t_rise1_prev);
  end
  always @(negedge PWM_OUT1) t_fall1 = $realtime;

  initial begin
`ifdef DUMPFILE
    $dumpfile(`DUMPFILE);
    // Only the two outputs. Dumping the whole hierarchy means logging a 12 MHz
    // clock for 60 ms -- 720,000 edges and a ~62 MB VCD, in a OneDrive folder.
    $dumpvars(1, PWM_OUT0, PWM_OUT1);
`endif
    $display("");
    $display("  clock = 12.000 MHz (83.333 ns period), buttons held idle");
    $display("  reporting measured period/duty of each output:");
    // long enough for >2 periods of the slowest plausible output
    #60_000_000;   // 60 ms
    $display("");
    $finish;
  end

endmodule
