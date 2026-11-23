`default_nettype none

// -----------------------------------------------------------------------------
// UART_Receiver : serial receiver, LSB first.
//
//     serial_in   : the RX line (asynchronous, goes through a 2-FF synchronizer).
//     data_out    : last good byte. Only changes when a frame was received
//                   correctly, so it stays stable while the next frame arrives.
//     data_valid  : 1-clk pulse, same cycle data_out gets the new byte.
//     frame_error : 1-clk pulse if the stop bit was not high. In that case
//                   data_out is NOT updated and data_valid is NOT pulsed.
//     busy        : high from start-bit detection until the middle of the stop
//                   bit (the receiver is already re-armed when busy falls).
//
// How a bit is decided (this is the "make sure the bit really was 0/1" part):
//   - A frame starts on a falling edge. That edge is the only timing reference
//     we get (there is no clock on the wire), so we count clocks from it.
//   - Every bit is judged around its middle, using 3 samples taken
//     VOTE_SPACING clocks apart, majority wins (2 of 3). A glitch shorter than
//     VOTE_SPACING clocks can only flip one sample, so it is voted away.
//   - The start bit is checked the same way, so a short glitch on an idle line
//     is rejected instead of being read as a frame.
// -----------------------------------------------------------------------------
module UART_Receiver #(
    parameter int CLK_FREQ_HZ  = 50_000_000,
    parameter int BAUD_RATE    = 115_200,
    parameter int DATA_WIDTH   = 8,
    parameter int VOTE_SPACING = 8            // clocks between the 3 voting samples (auto-reduced if the clock is slow)
) (
    input  logic                  clk,
    input  logic                  reset_n,    // async reset, active low
    input  logic                  serial_in,
    output logic [DATA_WIDTH-1:0] data_out,
    output logic                  data_valid,
    output logic                  frame_error,
    output logic                  busy
);

    // CLKS_PER_BIT better be greater or equal to 8
    localparam int CLKS_PER_BIT = (CLK_FREQ_HZ + BAUD_RATE / 2) / BAUD_RATE;
    localparam int HALF_BIT     = CLKS_PER_BIT / 2;

    localparam int VOTE_LIMIT   = CLKS_PER_BIT / 8;
    localparam int VOTE_GAP     = (VOTE_SPACING < VOTE_LIMIT) ? VOTE_SPACING : VOTE_LIMIT;
    localparam int VG           = (VOTE_GAP < 1) ? 1 : VOTE_GAP;

    localparam int CNT_W        = $clog2(CLKS_PER_BIT + 1);
    localparam int IDX_W        = $clog2(DATA_WIDTH + 1);

    localparam logic [CNT_W-1:0] CNT_MAX   = CNT_W'(CLKS_PER_BIT - 1);
    localparam logic [CNT_W-1:0] CNT_START = CNT_W'(HALF_BIT + VG - 1);
    localparam logic [IDX_W-1:0] LAST_IDX  = IDX_W'(DATA_WIDTH - 1);

    typedef enum logic [1:0] {
        IDLE,    // waiting for a falling edge
        START,   // waiting for the middle of the start bit to confirm it
        DATA,    // all data bits, one after another (replaces BIT0..BITn)
        STOP     // checking the stop bit
    } state_t;

    state_t state, next_state; 

    // Input synchronizer to avoid metastability.
    logic rx_meta;
    logic rx_sync;

    // History of rx_sync, Used for the 3-sample vote and for edge detection.
    logic [2*VG-1:0] rx_win;

    logic rx_fall;
    logic rx_bit;

    logic [CNT_W-1:0]      clk_cnt;
    logic [IDX_W-1:0]      bit_idx;
    logic [DATA_WIDTH-1:0] shift_reg;
    logic                  sample_tick;

    // ------------------------------------------------------- synchronizer ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            rx_win  <= '1;
        end
        else begin
            rx_meta <= serial_in;
            rx_sync <= rx_meta;
            rx_win  <= {rx_win[2*VG-2:0], rx_sync};
        end
    end

    assign rx_fall = rx_win[0] & ~rx_sync;

    // ---------------------------------------------------------- the vote -----
    // 3 samples: now, VG clocks ago, 2*VG clocks ago. Majority wins.
    // The middle sample is the center of the bit, the other two
    // are VG clocks before and after it.
    logic s_new, s_mid, s_old;
    assign s_new  = rx_sync;
    assign s_mid  = rx_win[VG-1];
    assign s_old  = rx_win[2*VG-1];
    assign rx_bit = (s_new & s_mid) | (s_new & s_old) | (s_mid & s_old);

    // ------------------------------------------------------ sample timing ----
    assign sample_tick = (state == START) ? (clk_cnt == CNT_START)
                                          : (clk_cnt == CNT_MAX);

    // ---------------------------------------------------------------- FSM ----
    always_ff @(posedge clk or negedge reset_n) begin // Sequential block
        if (!reset_n) state <= IDLE;
        else          state <= next_state;
    end

    always_comb begin // Combinational next-state block
        next_state = state;
        case (state)
            IDLE:  if (rx_fall)                             next_state = START;
            START: if (sample_tick)                         next_state = rx_bit ? IDLE : DATA; //> still high = it was a glitch, give up
            DATA:  if (sample_tick && bit_idx == LAST_IDX)  next_state = STOP;                 //> got the last data bit
            STOP:  if (sample_tick)                         next_state = IDLE;
            default:                                        next_state = IDLE;
        endcase
    end

    always_comb begin // Combinational output block
        busy = (state != IDLE);
    end

    // ----------------------------------------------------------- datapath ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            clk_cnt     <= '0;
            bit_idx     <= '0;
            shift_reg   <= '0;
            data_out    <= '0;
            data_valid  <= 1'b0;
            frame_error <= 1'b0;
        end
        else begin
            data_valid  <= 1'b0;
            frame_error <= 1'b0;

            if (state == IDLE || sample_tick) clk_cnt <= '0;
            else                              clk_cnt <= clk_cnt + 1'b1;

            case (state)
                IDLE: bit_idx <= '0;

                DATA: begin
                    if (sample_tick) begin
                        shift_reg <= {rx_bit, shift_reg[DATA_WIDTH-1:1]};
                        bit_idx   <= bit_idx + 1'b1;
                    end
                end

                STOP: begin
                    if (sample_tick) begin
                        data_out   <= shift_reg;
                        if (rx_bit) begin                // stop bit is high = good frame
                            data_valid <= 1'b1;
                        end
                        else begin                       // stop bit is low = something is wrong
                            frame_error <= 1'b1;
                        end
                    end
                end

                default: ;
            endcase
        end
    end

endmodule

`default_nettype wire
