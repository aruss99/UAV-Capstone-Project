`timescale 1ns/1ps

// FT Explorer fixed-point controller core.
//
// Boundary:
//   estimated state and pilot/autopilot demand in;
//   aileron, elevator and rudder demand out.
//
// This module intentionally excludes sensor acquisition, state estimation,
// PWM generation and motor control. Input words and outputs use signed Q5.10:
//   real_value = port_value / 1024.
// Phi commands/feedback and p/q/r are radians and radians/second. Theta and
// beta commands/feedback and all three outputs are degrees.
//
// The equations are the flat-gain reduction of DroneModelv7_FTX_ATT:
//   * clamped forward-Euler PI attitude loops at 50 Hz;
//   * 10 rad/s ZOH-discretized command filters;
//   * proportional p/q/r rate loops; and
//   * +/-12 degree surface saturation.
//
// Input load map:
//   0 p, 1 q, 2 r, 3 phi, 4 theta, 5 beta,
//   6 phi command, 7 theta command, 8 beta command.
// Load all nine words while busy is low, then pulse sample_valid. A single
// channel-indexed datapath, block RAM and signed serial multiplier serve all
// three axes so the controller fits the iCE40HX1K.

module ftx_controller_core (
    input  wire                    clk,
    input  wire                    reset,
    input  wire                    input_load_valid,
    input  wire [3:0]              input_index,
    input  wire signed [15:0]      input_value,
    input  wire                    sample_valid,

    output reg signed [15:0]       aileron_deg,
    output reg signed [15:0]       elevator_deg,
    output reg signed [15:0]       rudder_deg,
    output reg                     busy,
    output reg                     output_valid
);

    // Coefficients are signed Q5.14. Values are rounded from the authoritative
    // MATLAB setup (uav_setup_v7_ftx.m, Tc = 0.02 s).
    localparam signed [18:0] P_PHI       =  19'sd81920;   //  5.0
    localparam signed [18:0] P_THETA     =  19'sd819;     //  0.05
    localparam signed [18:0] P_BETA      = -19'sd819;     // -0.05
    localparam signed [18:0] I_PHI_DT    =  19'sd1638;    //  5.0 * 0.02
    localparam signed [18:0] I_THETA_DT  =  19'sd41;      //  0.125 * 0.02
    localparam signed [18:0] I_BETA_DT   = -19'sd20;      // -0.0625 * 0.02
    localparam signed [18:0] K_P         =  19'sd4989;    //  0.304503055367
    localparam signed [18:0] K_Q         = -19'sd41028;   // -2.50416204121
    localparam signed [18:0] K_R         =  19'sd147675;  //  9.01334991223
    localparam signed [18:0] LAG_A       =  19'sd13414;   // exp(-10*Tc)
    localparam signed [18:0] LAG_B       =  19'sd2970;    // 1-exp(-10*Tc)

    localparam [4:0]
        ST_INIT         = 5'd0,
        ST_IDLE         = 5'd1,
        ST_READ_RATE    = 5'd2,
        ST_CAPTURE_RATE = 5'd3,
        ST_CAPTURE_FB   = 5'd4,
        ST_CAPTURE_CMD  = 5'd5,
        ST_CAPTURE_I_LO = 5'd6,
        ST_CAPTURE_I_HI = 5'd7,
        ST_CAPTURE_LAG  = 5'd8,
        ST_P_START      = 5'd9,
        ST_P_WAIT       = 5'd10,
        ST_I_START      = 5'd11,
        ST_I_WAIT       = 5'd12,
        ST_PI           = 5'd13,
        ST_I_WRITE_LO   = 5'd14,
        ST_I_WRITE_HI   = 5'd15,
        ST_LAGA_START   = 5'd16,
        ST_LAGA_WAIT    = 5'd17,
        ST_LAGB_START   = 5'd18,
        ST_LAGB_WAIT    = 5'd19,
        ST_LAG          = 5'd20,
        ST_LAG_WRITE    = 5'd21,
        ST_RATE_START   = 5'd22,
        ST_RATE_WAIT    = 5'd23,
        ST_DONE         = 5'd24;

    reg [4:0] state;
    reg [1:0] channel;
    reg [4:0] init_address;

    // Addresses 0..8 hold the loaded sample. Addresses 9..14 hold three
    // 28-bit Q24 integrators as low/high word pairs; 15..17 hold lag states.
    // Synchronous read plus one write port maps to one iCE40 4-kbit RAM block.
    (* ram_style = "block" *) reg [15:0] controller_memory [0:31];
    reg [4:0] memory_read_address;
    reg [15:0] memory_read_data;

    wire [4:0] rate_address = {3'd0, channel};
    wire [4:0] feedback_address = 5'd3 + {3'd0, channel};
    wire [4:0] command_address = 5'd6 + {3'd0, channel};
    wire [4:0] integral_low_address = 5'd9 + {2'd0, channel, 1'b0};
    wire [4:0] integral_high_address = integral_low_address + 5'd1;
    wire [4:0] lag_address = 5'd15 + {3'd0, channel};

    reg signed [15:0] selected_rate_q10;
    reg signed [15:0] selected_feedback_q10;
    reg signed [27:0] selected_integral_q24;
    reg signed [15:0] selected_lag_q10;

    always @* begin
        case (state)
            ST_READ_RATE:    memory_read_address = rate_address;
            ST_CAPTURE_RATE: memory_read_address = feedback_address;
            ST_CAPTURE_FB:   memory_read_address = command_address;
            ST_CAPTURE_CMD:  memory_read_address = integral_low_address;
            ST_CAPTURE_I_LO: memory_read_address = integral_high_address;
            ST_CAPTURE_I_HI: memory_read_address = lag_address;
            default:         memory_read_address = 5'd0;
        endcase
    end

    always @(posedge clk) begin
        memory_read_data <= controller_memory[memory_read_address];

        if (state == ST_INIT)
            controller_memory[init_address] <= 16'd0;
        else if (!busy && input_load_valid && (input_index <= 4'd8))
            controller_memory[{1'b0, input_index}] <= input_value;
        else if (state == ST_I_WRITE_LO)
            controller_memory[integral_low_address] <= selected_integral_q24[15:0];
        else if (state == ST_I_WRITE_HI)
            controller_memory[integral_high_address]
                <= {{4{selected_integral_q24[27]}}, selected_integral_q24[27:16]};
        else if (state == ST_LAG_WRITE)
            controller_memory[lag_address] <= selected_lag_q10;
    end

    reg signed [15:0] error_q10;
    reg signed [15:0] proportional_q10;
    reg signed [15:0] command_q10;
    reg signed [15:0] lag_term_a_q10;

    reg                    multiplier_start;
    reg signed [15:0]      multiplier_x;
    reg signed [18:0]      multiplier_coefficient;
    wire                   multiplier_done;
    wire signed [34:0]     multiplier_product;

    ftx_serial_multiplier shared_multiplier (
        .clk(clk),
        .reset(reset),
        .start(multiplier_start),
        .x(multiplier_x),
        .coefficient(multiplier_coefficient),
        .done(multiplier_done),
        .product(multiplier_product)
    );

    reg signed [18:0] selected_p_coefficient;
    reg signed [18:0] selected_i_coefficient;
    reg signed [18:0] selected_rate_coefficient;
    always @* begin
        case (channel)
            2'd0: begin
                selected_p_coefficient    = P_PHI;
                selected_i_coefficient    = I_PHI_DT;
                selected_rate_coefficient = K_P;
            end
            2'd1: begin
                selected_p_coefficient    = P_THETA;
                selected_i_coefficient    = I_THETA_DT;
                selected_rate_coefficient = K_Q;
            end
            default: begin
                selected_p_coefficient    = P_BETA;
                selected_i_coefficient    = I_BETA_DT;
                selected_rate_coefficient = K_R;
            end
        endcase
    end

    wire signed [16:0] loaded_input_difference_q10 =
        $signed({memory_read_data[15], memory_read_data})
        - $signed({selected_feedback_q10[15], selected_feedback_q10});
    wire signed [15:0] loaded_error_q10 =
        (loaded_input_difference_q10[16] != loaded_input_difference_q10[15])
            ? (loaded_input_difference_q10[16] ? -16'sd32768 : 16'sh7fff)
            : loaded_input_difference_q10[15:0];

    wire signed [16:0] rate_difference_q10 =
        $signed({selected_lag_q10[15], selected_lag_q10})
        - $signed({selected_rate_q10[15], selected_rate_q10});
    wire signed [15:0] selected_rate_error_q10 =
        (rate_difference_q10[16] != rate_difference_q10[15])
            ? (rate_difference_q10[16] ? -16'sd32768 : 16'sh7fff)
            : rate_difference_q10[15:0];

    // Symmetric round-to-nearest conversion from Q24 to Q10. The negative
    // exact-half case preserves ties away from zero.
    wire signed [20:0] product_shifted_q10 = multiplier_product[34:14];
    wire product_round_up = multiplier_product[13]
        && !(multiplier_product[34] && (multiplier_product[12:0] == 13'd0));
    wire signed [21:0] product_rounded_q10 =
        $signed({product_shifted_q10[20], product_shifted_q10})
        + $signed({21'd0, product_round_up});
    wire signed [15:0] converted_product_q10 =
        (product_rounded_q10 > 22'sd32767) ? 16'sh7fff :
        (product_rounded_q10 < -22'sd32768) ? -16'sd32768 :
        product_rounded_q10[15:0];

    wire signed [14:0] selected_integral_q10 =
        {selected_integral_q24[27], selected_integral_q24[27:14]};
    wire signed [16:0] pi_sum_q10 =
        $signed({proportional_q10[15], proportional_q10})
        + $signed({{2{selected_integral_q10[14]}}, selected_integral_q10});
    wire signed [15:0] clamped_command_q10 =
        (pi_sum_q10 > 17'sd2048) ? 16'sd2048 :
        (pi_sum_q10 < -17'sd2048) ? -16'sd2048 : pi_sum_q10[15:0];

    wire signed [28:0] integral_candidate_q24 =
        $signed({selected_integral_q24[27], selected_integral_q24})
        + $signed(multiplier_product[28:0]);
    wire signed [27:0] clamped_integral_q24 =
        (integral_candidate_q24 > 29'sd67108864) ? 28'sd67108864 :
        (integral_candidate_q24 < -29'sd67108864) ? -28'sd67108864 :
        integral_candidate_q24[27:0];
    wire integral_drives_further =
        ((pi_sum_q10 > 17'sd2048) && !multiplier_product[34]
                                      && (multiplier_product != 35'sd0))
        || ((pi_sum_q10 < -17'sd2048) && multiplier_product[34]);
    wire signed [27:0] selected_integral_next_q24 =
        integral_drives_further ? selected_integral_q24 : clamped_integral_q24;

    wire signed [16:0] lag_sum_q10 =
        $signed({lag_term_a_q10[15], lag_term_a_q10})
        + $signed({converted_product_q10[15], converted_product_q10});
    wire signed [15:0] next_lag_q10 =
        (lag_sum_q10[16] != lag_sum_q10[15])
            ? (lag_sum_q10[16] ? -16'sd32768 : 16'sh7fff)
            : lag_sum_q10[15:0];

    wire signed [15:0] clamped_surface_q10 =
        (converted_product_q10 > 16'sd12288) ? 16'sd12288 :
        (converted_product_q10 < -16'sd12288) ? -16'sd12288 :
        converted_product_q10;

    always @(posedge clk) begin
        if (reset) begin
            state             <= ST_INIT;
            channel           <= 2'd0;
            init_address      <= 5'd9;
            busy              <= 1'b1;
            output_valid      <= 1'b0;
            multiplier_start  <= 1'b0;
            aileron_deg       <= 16'sd0;
            elevator_deg      <= 16'sd0;
            rudder_deg        <= 16'sd0;
        end else begin
            multiplier_start <= 1'b0;
            output_valid     <= 1'b0;

            case (state)
                ST_INIT: begin
                    if (init_address == 5'd17) begin
                        busy  <= 1'b0;
                        state <= ST_IDLE;
                    end else
                        init_address <= init_address + 5'd1;
                end

                ST_IDLE: begin
                    busy <= 1'b0;
                    if (sample_valid) begin
                        channel <= 2'd0;
                        busy    <= 1'b1;
                        state   <= ST_READ_RATE;
                    end
                end

                ST_READ_RATE: state <= ST_CAPTURE_RATE;
                ST_CAPTURE_RATE: begin
                    selected_rate_q10 <= memory_read_data;
                    state <= ST_CAPTURE_FB;
                end
                ST_CAPTURE_FB: begin
                    selected_feedback_q10 <= memory_read_data;
                    state <= ST_CAPTURE_CMD;
                end
                ST_CAPTURE_CMD: begin
                    error_q10 <= loaded_error_q10;
                    state <= ST_CAPTURE_I_LO;
                end
                ST_CAPTURE_I_LO: begin
                    selected_integral_q24[15:0] <= memory_read_data;
                    state <= ST_CAPTURE_I_HI;
                end
                ST_CAPTURE_I_HI: begin
                    selected_integral_q24[27:16] <= memory_read_data[11:0];
                    state <= ST_CAPTURE_LAG;
                end
                ST_CAPTURE_LAG: begin
                    selected_lag_q10 <= memory_read_data;
                    state <= ST_P_START;
                end

                ST_P_START: begin
                    multiplier_x           <= error_q10;
                    multiplier_coefficient <= selected_p_coefficient;
                    multiplier_start       <= 1'b1;
                    state                  <= ST_P_WAIT;
                end
                ST_P_WAIT: if (multiplier_done) begin
                    proportional_q10 <= converted_product_q10;
                    state <= ST_I_START;
                end
                ST_I_START: begin
                    multiplier_x           <= error_q10;
                    multiplier_coefficient <= selected_i_coefficient;
                    multiplier_start       <= 1'b1;
                    state                  <= ST_I_WAIT;
                end
                ST_I_WAIT: if (multiplier_done)
                    state <= ST_PI;

                ST_PI: begin
                    command_q10           <= clamped_command_q10;
                    selected_integral_q24 <= selected_integral_next_q24;
                    state                 <= ST_I_WRITE_LO;
                end
                ST_I_WRITE_LO: state <= ST_I_WRITE_HI;
                ST_I_WRITE_HI: state <= ST_LAGA_START;

                ST_LAGA_START: begin
                    multiplier_x           <= selected_lag_q10;
                    multiplier_coefficient <= LAG_A;
                    multiplier_start       <= 1'b1;
                    state                  <= ST_LAGA_WAIT;
                end
                ST_LAGA_WAIT: if (multiplier_done) begin
                    lag_term_a_q10 <= converted_product_q10;
                    state <= ST_LAGB_START;
                end
                ST_LAGB_START: begin
                    multiplier_x           <= command_q10;
                    multiplier_coefficient <= LAG_B;
                    multiplier_start       <= 1'b1;
                    state                  <= ST_LAGB_WAIT;
                end
                ST_LAGB_WAIT: if (multiplier_done)
                    state <= ST_LAG;

                ST_LAG: begin
                    selected_lag_q10 <= next_lag_q10;
                    state <= ST_LAG_WRITE;
                end
                ST_LAG_WRITE: state <= ST_RATE_START;

                ST_RATE_START: begin
                    multiplier_x           <= selected_rate_error_q10;
                    multiplier_coefficient <= selected_rate_coefficient;
                    multiplier_start       <= 1'b1;
                    state                  <= ST_RATE_WAIT;
                end
                ST_RATE_WAIT: if (multiplier_done) begin
                    case (channel)
                        2'd0: aileron_deg  <= clamped_surface_q10;
                        2'd1: elevator_deg <= clamped_surface_q10;
                        default: rudder_deg <= clamped_surface_q10;
                    endcase
                    if (channel == 2'd2)
                        state <= ST_DONE;
                    else begin
                        channel <= channel + 2'd1;
                        state   <= ST_READ_RATE;
                    end
                end

                ST_DONE: begin
                    output_valid <= 1'b1;
                    busy         <= 1'b0;
                    state        <= ST_IDLE;
                end

                default: state <= ST_INIT;
            endcase
        end
    end

endmodule
