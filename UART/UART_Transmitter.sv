`default_nettype none

// -----------------------------------------------------------------------------
// UART_Transmitter : serial transmitter, LSB first.
//
//     start      : pulse for 1 clk while busy == 0 to send data_in.
//                  Ignored while busy == 1. (If you hold it high, the byte is
//                  sent again as soon as the previous frame ends - use a pulse.)
//     data_in    : latched on the clk edge where start is accepted, so it only
//                  has to be valid in that one cycle.
//     serial_out : the TX line. Registered (glitch free), idles high.
//     busy       : high from the cycle after start is accepted until the end
//                  of the stop bit.
//     done       : 1-clk pulse in the first cycle after the stop bit finished
//                  (the same cycle busy falls).
// -----------------------------------------------------------------------------
module UART_Transmitter #(
    parameter int CLK_FREQ_HZ = 50_000_000,
    parameter int BAUD_RATE   = 115_200,
    parameter int DATA_WIDTH  = 8
) (
    input  logic                  clk,
    input  logic                  reset_n,   // async reset, active low
    input  logic                  start,
    input  logic [DATA_WIDTH-1:0] data_in,
    output logic                  serial_out,
    output logic                  busy,
    output logic                  done
);

    // CLKS_PER_BIT should be greater or equal to 2
    localparam int CLKS_PER_BIT = (CLK_FREQ_HZ + BAUD_RATE / 2) / BAUD_RATE;

    localparam int CNT_W        = $clog2(CLKS_PER_BIT + 1);
    localparam int IDX_W        = $clog2(DATA_WIDTH + 1);

    localparam logic [CNT_W-1:0] CNT_MAX  = CNT_W'(CLKS_PER_BIT - 1);
    localparam logic [IDX_W-1:0] LAST_IDX = IDX_W'(DATA_WIDTH - 1);

    typedef enum logic [1:0] {
        IDLE,    // line high, waiting for start
        START,   // start bit (line low) for 1 bit time
        DATA,    // all data bits, one after another
        STOP     // stop bit (line high) for 1 bit time
    } state_t;

    state_t state, next_state;

    logic [CNT_W-1:0]      clk_cnt;
    logic [IDX_W-1:0]      bit_idx;
    logic [DATA_WIDTH-1:0] shift_reg;
    logic [DATA_WIDTH-1:0] shift_next;
    logic                  tx_q;
    logic                  bit_tick;

    assign bit_tick   = (clk_cnt == CNT_MAX);
    assign shift_next = shift_reg >> 1;

    // ---------------------------------------------------------------- FSM ----
    always_ff @(posedge clk or negedge reset_n) begin // Sequential block
        if (!reset_n) state <= IDLE;
        else          state <= next_state;
    end

    always_comb begin // Combinational next-state block
        next_state = state;                    //> default: stay where we are
        case (state)
            IDLE:    if (start)                           next_state = START;
            START:   if (bit_tick)                        next_state = DATA;
            DATA:    if (bit_tick && bit_idx == LAST_IDX) next_state = STOP;
            STOP:    if (bit_tick)                        next_state = IDLE;
            default:                                      next_state = IDLE;
        endcase
    end

    always_comb begin // Combinational output block
        busy = (state != IDLE);
    end

    // ----------------------------------------------------------- datapath ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            clk_cnt   <= '0;
            bit_idx   <= '0;
            shift_reg <= '0;
            tx_q      <= 1'b1;
            done      <= 1'b0;
        end
        else begin
            done <= 1'b0;

            if (state == IDLE || bit_tick) clk_cnt <= '0;
            else                           clk_cnt <= clk_cnt + 1'b1;

            // tx_q changes on the same clock edge as the state, so the wire
            // and the FSM always agree on which bit we are in.
            case (state)
                IDLE: begin
                    bit_idx <= '0;
                    if (start) begin
                        shift_reg <= data_in;
                        tx_q      <= 1'b0;
                    end
                end

                START: begin
                    if (bit_tick) tx_q <= shift_reg[0];
                end

                DATA: begin
                    if (bit_tick) begin
                        if (bit_idx == LAST_IDX) begin
                            tx_q <= 1'b1;
                        end
                        else begin
                            shift_reg <= shift_next;
                            tx_q      <= shift_next[0];
                            bit_idx   <= bit_idx + 1'b1;
                        end
                    end
                end

                STOP: begin
                    if (bit_tick) done <= 1'b1;
                end

                default: ;
            endcase
        end
    end

    assign serial_out = tx_q;

endmodule

`default_nettype wire
