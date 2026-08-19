// shift_register.v
//
// An 8-bit shift register with a synchronous active-low reset.
//
// STATUS: this compiles, elaborates and simulates. It is NOT a correct I2C or
// SPI shifter -- there is no protocol framing, and toggling `scl` in the same
// branch that shifts data is not a valid serial clock. Treat it as a fragment,
// not a working interface.

module shift8 (
    input        reset,
    input        send_message,
    input        clk,
    input        serial_in,
    output reg   scl,
    output       serial_out
);

    reg [3:0] count;
    reg [7:0] bits;

    assign serial_out = bits[7];

    always @(posedge clk) begin
        if (!reset) begin
            scl   <= 1'b1;
            count <= 4'd0;
            bits  <= 8'b0000_0000;
        end else if (count >= 12 && send_message == 1'b1) begin
            scl     <= ~scl;
            bits    <= {bits[6:0], serial_in};
            count   <= 4'd0;
        end else begin
            count <= count + 1'b1;
        end
    end

endmodule
