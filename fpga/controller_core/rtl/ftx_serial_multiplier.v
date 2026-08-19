`timescale 1ns/1ps

// Signed 16-by-19 bit serial multiplier.
//
// The iCE40HX1K has no DSP block. A parallel multiplier consumes most of the
// device, while the 50 Hz controller has 240,000 clock cycles available at the
// custom board's 12 MHz clock. This unit spends 19 cycles per product and is
// shared by every controller operation. The final coefficient bit is treated
// as a negative two's-complement weight, avoiding magnitude/negation datapaths.

module ftx_serial_multiplier (
    input  wire                    clk,
    input  wire                    reset,
    input  wire                    start,
    input  wire signed [15:0]      x,
    input  wire signed [18:0]      coefficient,
    output reg                     done,
    output reg signed [34:0]       product
);

    reg                    busy;
    reg signed [34:0]      accumulator;
    reg signed [34:0]      multiplicand;
    reg [18:0]             multiplier;
    reg [4:0]              bit_count;
    reg signed [34:0]      accumulator_next;

    always @* begin
        accumulator_next = accumulator;
        if (multiplier[0]) begin
            if (bit_count == 5'd18)
                accumulator_next = accumulator - multiplicand;
            else
                accumulator_next = accumulator + multiplicand;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            busy    <= 1'b0;
            done    <= 1'b0;
            product <= 35'sd0;
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                busy           <= 1'b1;
                accumulator    <= 35'sd0;
                multiplicand   <= {{19{x[15]}}, x};
                multiplier     <= coefficient;
                bit_count      <= 5'd0;
            end else if (busy) begin
                accumulator <= accumulator_next;
                if (bit_count == 5'd18) begin
                    product <= accumulator_next;
                    busy    <= 1'b0;
                    done    <= 1'b1;
                end else begin
                    multiplicand <= multiplicand <<< 1;
                    multiplier   <= multiplier >> 1;
                    bit_count    <= bit_count + 5'd1;
                end
            end
        end
    end

endmodule
