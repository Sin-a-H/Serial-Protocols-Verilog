`default_nettype none

// -----------------------------------------------------------------------------
// UART : demo / reference top level.
//
// What it does:
//   every SEND_PERIOD_MS it sends 4 bytes (DE AD BE EF) through the transmitter.
//   The TX line is looped back into the receiver, which should then pulse
//   rx_valid once per byte and show the byte on rx_data.
// -----------------------------------------------------------------------------
module UART #(
    parameter int CLK_FREQ_HZ    = 50_000_000,
    parameter int BAUD_RATE      = 115_200,
    parameter int SEND_PERIOD_MS = 100
) (
    input  wire       CLK,
    input  wire       reset_n,         // reset button, active low

    output wire       tx,

    output wire       tx_busy,
    output wire       tx_done,
    output wire       rx_busy,
    output wire       rx_valid,
    output wire       rx_frame_error,
    output wire [7:0] rx_data
);

    // -------------------------------------------------------------- reset ----
    always_ff @(posedge CLK or negedge reset_n) begin
        if (!reset_n) rst_sync <= 2'b00;
        else          rst_sync <= {rst_sync[0], 1'b1};
    end

    assign rst_n = rst_sync[1];

    // ---------------------------------------------------- the two modules ----
    wire       tx_line;
    logic      tx_start;
    logic [7:0] tx_data;
    wire       tx_busy_w;
    wire       tx_done_w;
    wire       rx_valid_w;
    wire       rx_ferr_w;

    UART_Transmitter #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE  (BAUD_RATE),
        .DATA_WIDTH (8)
    ) u_tx (
        .clk       (CLK),
        .reset_n   (rst_n),
        .start     (tx_start),
        .data_in   (tx_data),
        .serial_out(tx_line),
        .busy      (tx_busy_w),
        .done      (tx_done_w)
    );

    UART_Receiver #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE  (BAUD_RATE),
        .DATA_WIDTH (8)
    ) u_rx (
        .clk        (CLK),
        .reset_n    (rst_n),
        .serial_in  (tx_line),
        .data_out   (rx_data),
        .data_valid (rx_valid_w),
        .frame_error(rx_ferr_w),
        .busy       (rx_busy)
    );

    assign tx      = tx_line;       // For debugging
    assign tx_busy = tx_busy_w;

    // -------------------------------------------------------- the sender -----
    // A tiny FSM that plays the "user" of the transmitter
    // wait a while, then send 4 bytes one after another, then wait again.
    localparam int PERIOD_CLKS = (CLK_FREQ_HZ / 1000) * SEND_PERIOD_MS;  //> pause length in clocks
    localparam int PERIOD_W    = $clog2(PERIOD_CLKS + 1);
    localparam logic [PERIOD_W-1:0] PERIOD_MAX = PERIOD_W'(PERIOD_CLKS - 1);

    typedef enum logic [1:0] {
        WAIT_PERIOD,
        SEND_REQ,
        SEND_WAIT
    } send_state_t;

    send_state_t         send_state, send_next;
    logic [PERIOD_W-1:0] period_cnt;
    logic [1:0]          byte_idx;

    always_ff @(posedge CLK or negedge rst_n) begin // Sequential block
        if (!rst_n) send_state <= WAIT_PERIOD;
        else        send_state <= send_next;
    end

    always_comb begin // Combinational next-state block
        send_next = send_state;
        case (send_state)
            WAIT_PERIOD: if (period_cnt == PERIOD_MAX) send_next = SEND_REQ;
            SEND_REQ:    if (!tx_busy_w)               send_next = SEND_WAIT;
            SEND_WAIT:   if (tx_done_w)                send_next = (byte_idx == 2'd3) ? WAIT_PERIOD
                                                                                      : SEND_REQ;
            default:                                   send_next = WAIT_PERIOD;
        endcase
    end

    always_comb begin // Combinational output block
        tx_start = (send_state == SEND_REQ) && !tx_busy_w;

        case (byte_idx)
            2'd0:    tx_data = 8'hDE;
            2'd1:    tx_data = 8'hAD;
            2'd2:    tx_data = 8'hBE;
            default: tx_data = 8'hEF;
        endcase
    end

    always_ff @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            period_cnt <= '0;
            byte_idx   <= '0;
        end
        else begin
            // pause timer only runs while we wait; it restarts from 0 afterwards
            if (send_state == WAIT_PERIOD) period_cnt <= period_cnt + 1'b1;
            else                           period_cnt <= '0;

            // move on to the next byte each time one finishes (wraps 3 -> 0)
            if (send_state == SEND_WAIT && tx_done_w) byte_idx <= byte_idx + 1'b1;
        end
    end

    // -------------------------------------------------- pulse stretching -----
    //> tx_done / rx_valid / rx_frame_error are 1 clock wide = 20 ns at 50 MHz.
    //> A logic analyzer would never see that, so for this demo each one is
    //> held high for about one bit time (~8.7 us at 115200).
    //> Not needed in a real design - there you just use the 1-clk pulse.
    localparam int STRETCH_CLKS = CLK_FREQ_HZ / BAUD_RATE;

    UART_PulseStretch #(.HOLD_CLKS(STRETCH_CLKS)) u_st_txdone (
        .clk(CLK), .reset_n(rst_n), .pulse_in(tx_done_w),  .pulse_out(tx_done));
    UART_PulseStretch #(.HOLD_CLKS(STRETCH_CLKS)) u_st_rxvalid (
        .clk(CLK), .reset_n(rst_n), .pulse_in(rx_valid_w), .pulse_out(rx_valid));
    UART_PulseStretch #(.HOLD_CLKS(STRETCH_CLKS)) u_st_rxferr (
        .clk(CLK), .reset_n(rst_n), .pulse_in(rx_ferr_w),  .pulse_out(rx_frame_error));

endmodule


// -----------------------------------------------------------------------------
// Demo helper only (not needed by the TX / RX modules): makes a 1-clk pulse
// last HOLD_CLKS clocks so slow instruments can see it.
// -----------------------------------------------------------------------------
module UART_PulseStretch #(
    parameter int HOLD_CLKS = 434
) (
    input  wire clk,
    input  wire reset_n,
    input  wire pulse_in,
    output wire pulse_out
);
    localparam int W = $clog2(HOLD_CLKS + 1);

    logic [W-1:0] cnt;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)         cnt <= '0;
        else if (pulse_in)    cnt <= W'(HOLD_CLKS);
        else if (cnt != '0)   cnt <= cnt - 1'b1;
    end

    assign pulse_out = (cnt != '0);
endmodule

`default_nettype wire
