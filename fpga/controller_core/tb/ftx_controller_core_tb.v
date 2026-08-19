`timescale 1ns/1ps

module ftx_controller_core_tb;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg reset = 1'b1;
    reg input_load_valid = 1'b0;
    reg [3:0] input_index = 4'd0;
    reg signed [15:0] input_value = 16'sd0;
    reg sample_valid = 1'b0;

    reg signed [15:0] p_rad_s;
    reg signed [15:0] q_rad_s;
    reg signed [15:0] r_rad_s;
    reg signed [15:0] phi_rad;
    reg signed [15:0] theta_deg;
    reg signed [15:0] beta_deg;
    reg signed [15:0] phi_command_rad;
    reg signed [15:0] theta_command_deg;
    reg signed [15:0] beta_command_deg;

    wire signed [15:0] aileron_deg;
    wire signed [15:0] elevator_deg;
    wire signed [15:0] rudder_deg;
    wire busy;
    wire output_valid;

    integer input_file;
    integer output_file;
    integer scan_count;
    integer vector_count;
    integer latency_cycles;
    integer maximum_latency_cycles;
    reg [1023:0] input_path;
    reg [1023:0] output_path;

    ftx_controller_core dut (
        .clk(clk),
        .reset(reset),
        .input_load_valid(input_load_valid),
        .input_index(input_index),
        .input_value(input_value),
        .sample_valid(sample_valid),
        .aileron_deg(aileron_deg),
        .elevator_deg(elevator_deg),
        .rudder_deg(rudder_deg),
        .busy(busy),
        .output_valid(output_valid)
    );

    task load_word;
        input [3:0] index;
        input signed [15:0] value;
        begin
            @(posedge clk);
            input_load_valid <= 1'b1;
            input_index      <= index;
            input_value      <= value;
        end
    endtask

    initial begin
        if (!$value$plusargs("VECTORS=%s", input_path)) begin
            $fatal(1, "ERROR: +VECTORS=<path> is required");
        end
        if (!$value$plusargs("ACTUAL=%s", output_path)) begin
            $fatal(1, "ERROR: +ACTUAL=<path> is required");
        end

        input_file = $fopen(input_path, "r");
        output_file = $fopen(output_path, "w");
        if (input_file == 0 || output_file == 0) begin
            $fatal(1, "ERROR: could not open vector or output file");
        end

        vector_count = 0;
        maximum_latency_cycles = 0;
        repeat (5) @(posedge clk);
        reset <= 1'b0;
        wait (busy == 1'b0);

        while (!$feof(input_file)) begin
            scan_count = $fscanf(input_file, "%d %d %d %d %d %d %d %d %d\n",
                p_rad_s, q_rad_s, r_rad_s,
                phi_rad, theta_deg, beta_deg,
                phi_command_rad, theta_command_deg, beta_command_deg);
            if (scan_count == 9) begin
                load_word(4'd0, p_rad_s);
                load_word(4'd1, q_rad_s);
                load_word(4'd2, r_rad_s);
                load_word(4'd3, phi_rad);
                load_word(4'd4, theta_deg);
                load_word(4'd5, beta_deg);
                load_word(4'd6, phi_command_rad);
                load_word(4'd7, theta_command_deg);
                load_word(4'd8, beta_command_deg);
                @(posedge clk);
                input_load_valid <= 1'b0;
                sample_valid     <= 1'b1;
                @(posedge clk);
                sample_valid <= 1'b0;
                latency_cycles = 0;
                while (output_valid != 1'b1) begin
                    @(posedge clk);
                    latency_cycles = latency_cycles + 1;
                end
                if (latency_cycles > maximum_latency_cycles)
                    maximum_latency_cycles = latency_cycles;
                $fwrite(output_file, "%0d %0d %0d\n",
                    $signed(aileron_deg), $signed(elevator_deg), $signed(rudder_deg));
                vector_count = vector_count + 1;
                @(posedge clk);
            end
        end

        $fclose(input_file);
        $fclose(output_file);
        $display("RTL_VECTORS=%0d", vector_count);
        $display("MAX_LATENCY_CYCLES=%0d", maximum_latency_cycles);
        $finish;
    end
endmodule
