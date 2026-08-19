`timescale 1ns/1ps

module ftx_controller_board_top_tb;
    reg sys_clk = 1'b0;
    always #42 sys_clk = ~sys_clk; // approximately 12 MHz

    reg scan_clk_pin = 1'b0;
    reg scan_data_in_pin = 1'b0;
    reg scan_select_n_pin = 1'b1;
    wire scan_data_out_pin;

    reg [47:0] result_bits;
    integer bit_index;

    ftx_controller_board_top dut (
        .sys_clk(sys_clk),
        .scan_clk_pin(scan_clk_pin),
        .scan_data_in_pin(scan_data_in_pin),
        .scan_select_n_pin(scan_select_n_pin),
        .scan_data_out_pin(scan_data_out_pin)
    );

    task shift_input_word;
        input signed [15:0] value;
        integer index;
        begin
            for (index = 0; index < 16; index = index + 1) begin
                scan_data_in_pin = value[index];
                #500 scan_clk_pin = 1'b1;
                #500 scan_clk_pin = 1'b0;
            end
        end
    endtask

    initial begin
        // First deterministic regression vector. Expected Q5.10 result:
        // aileron=-5, elevator=65, rudder=-370.
        #25000;
        scan_select_n_pin = 1'b0;
        #1000;
        shift_input_word(16'sd15);
        shift_input_word(16'sd26);
        shift_input_word(16'sd41);
        shift_input_word(16'sd0);
        shift_input_word(16'sd1638);
        shift_input_word(16'sd0);
        shift_input_word(16'sd0);
        shift_input_word(16'sd1638);
        shift_input_word(16'sd0);
        #1000 scan_select_n_pin = 1'b1;

        wait (scan_data_out_pin == 1'b1);
        #1000 scan_select_n_pin = 1'b0;
        #1000;
        for (bit_index = 0; bit_index < 48; bit_index = bit_index + 1) begin
            scan_clk_pin = 1'b1;
            #250 result_bits[bit_index] = scan_data_out_pin;
            #250 scan_clk_pin = 1'b0;
            #500;
        end
        scan_select_n_pin = 1'b1;

        if (($signed(result_bits[15:0]) != -16'sd5)
            || ($signed(result_bits[31:16]) != 16'sd65)
            || ($signed(result_bits[47:32]) != -16'sd370)) begin
            $fatal(1, "SCAN_WRAPPER_FAIL da=%0d de=%0d dr=%0d",
                $signed(result_bits[15:0]),
                $signed(result_bits[31:16]),
                $signed(result_bits[47:32]));
        end

        $display("SCAN_WRAPPER_MATCH=PASS da=%0d de=%0d dr=%0d",
            $signed(result_bits[15:0]),
            $signed(result_bits[31:16]),
            $signed(result_bits[47:32]));
        $finish;
    end
endmodule
