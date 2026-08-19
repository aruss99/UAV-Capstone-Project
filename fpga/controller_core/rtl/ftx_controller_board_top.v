`timescale 1ns/1ps

// Custom-PCB compile wrapper for ftx_controller_core.
//
// This is deliberately a compact synchronous verification interface, not a
// flight I/O layer. It keeps controller inputs and outputs live through place
// and route without claiming BMI323/BMP390 acquisition, estimation or PWM.
//
// Pin reuse:
//   RX0  -> scan clock, RX1 -> scan data in,
//   TX0  -> scan data out, SBUS -> active-low scan select.
//
// Protocol (external scan clock must be <= 1 MHz):
//   1. With result_ready low on scan_data_out while select is high, select low
//      and shift nine signed 16-bit Q5.10 input words LSB-first in the core's
//      documented input-index order. Raise select after exactly 144 bits.
//   2. Wait for scan_data_out to become high while select is high.
//   3. Select low and sample 48 output bits LSB-first on scan-clock rising
//      edges: aileron, elevator, rudder. Raise select to clear result_ready.

module ftx_controller_board_top (
    input  wire sys_clk,
    input  wire scan_clk_pin,
    input  wire scan_data_in_pin,
    input  wire scan_select_n_pin,
    output wire scan_data_out_pin
);

    reg [7:0] power_on_count = 8'd0;
    always @(posedge sys_clk) begin
        if (!(&power_on_count))
            power_on_count <= power_on_count + 8'd1;
    end
    wire reset = !(&power_on_count);

    reg [2:0] scan_clk_sync;
    reg [1:0] scan_data_sync;
    reg [2:0] scan_select_sync;
    wire scan_clk_rising = (scan_clk_sync[2:1] == 2'b01);
    wire scan_clk_falling = (scan_clk_sync[2:1] == 2'b10);
    wire scan_select_falling = (scan_select_sync[2:1] == 2'b10);
    wire scan_select_rising = (scan_select_sync[2:1] == 2'b01);
    wire scan_selected = !scan_select_sync[2];

    reg [15:0] receive_shift;
    reg [3:0] receive_bit_count;
    reg [3:0] receive_word_count;
    reg [5:0] transmit_bit_count;
    reg result_ready;

    reg core_input_load_valid;
    reg [3:0] core_input_index;
    reg signed [15:0] core_input_value;
    reg core_sample_valid;
    wire core_busy;
    wire core_output_valid;
    wire signed [15:0] aileron_value;
    wire signed [15:0] elevator_value;
    wire signed [15:0] rudder_value;

    ftx_controller_core controller (
        .clk(sys_clk),
        .reset(reset),
        .input_load_valid(core_input_load_valid),
        .input_index(core_input_index),
        .input_value(core_input_value),
        .sample_valid(core_sample_valid),
        .aileron_deg(aileron_value),
        .elevator_deg(elevator_value),
        .rudder_deg(rudder_value),
        .busy(core_busy),
        .output_valid(core_output_valid)
    );

    reg transmit_bit;
    always @* begin
        if (transmit_bit_count < 6'd16)
            transmit_bit = aileron_value[transmit_bit_count[3:0]];
        else if (transmit_bit_count < 6'd32)
            transmit_bit = elevator_value[transmit_bit_count[3:0]];
        else
            transmit_bit = rudder_value[transmit_bit_count[3:0]];
    end
    assign scan_data_out_pin = scan_select_sync[2]
        ? result_ready
        : (result_ready ? transmit_bit : 1'b0);

    always @(posedge sys_clk) begin
        scan_clk_sync    <= {scan_clk_sync[1:0], scan_clk_pin};
        scan_data_sync   <= {scan_data_sync[0], scan_data_in_pin};
        scan_select_sync <= {scan_select_sync[1:0], scan_select_n_pin};

        if (reset) begin
            scan_clk_sync        <= 3'b000;
            scan_data_sync       <= 2'b00;
            scan_select_sync     <= 3'b111;
            receive_shift        <= 16'd0;
            receive_bit_count    <= 4'd0;
            receive_word_count   <= 4'd0;
            transmit_bit_count   <= 6'd0;
            result_ready         <= 1'b0;
            core_input_load_valid <= 1'b0;
            core_input_index     <= 4'd0;
            core_input_value     <= 16'sd0;
            core_sample_valid    <= 1'b0;
        end else begin
            core_input_load_valid <= 1'b0;
            core_sample_valid     <= 1'b0;

            if (core_output_valid)
                result_ready <= 1'b1;

            if (scan_select_falling) begin
                transmit_bit_count <= 6'd0;
                if (!result_ready) begin
                    receive_shift      <= 16'd0;
                    receive_bit_count  <= 4'd0;
                    receive_word_count <= 4'd0;
                end
            end

            if (scan_selected && scan_clk_rising && !result_ready && !core_busy) begin
                receive_shift <= {scan_data_sync[1], receive_shift[15:1]};
                if (receive_bit_count == 4'd15) begin
                    core_input_index      <= receive_word_count;
                    core_input_value      <= {scan_data_sync[1], receive_shift[15:1]};
                    core_input_load_valid <= 1'b1;
                    receive_bit_count     <= 4'd0;
                    receive_word_count    <= receive_word_count + 4'd1;
                end else
                    receive_bit_count <= receive_bit_count + 4'd1;
            end

            if (scan_selected && scan_clk_falling && result_ready
                && (transmit_bit_count < 6'd47))
                transmit_bit_count <= transmit_bit_count + 6'd1;

            if (scan_select_rising) begin
                if (result_ready)
                    result_ready <= 1'b0;
                else if ((receive_word_count == 4'd9)
                         && (receive_bit_count == 4'd0) && !core_busy)
                    core_sample_valid <= 1'b1;
            end
        end
    end

endmodule
