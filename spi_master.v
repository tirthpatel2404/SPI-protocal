`timescale 1ns/1ps
//=====================================================================
// spi_master.v -- parameterised SPI master, all four CPOL/CPHA modes
//
// Design decisions you must be able to defend:
//
//  1. ONE clock domain. SCLK is a REGISTERED OUTPUT, never a clock
//     inside this design. All sequential logic runs on `clk` and is
//     gated by a single-cycle enable strobe (half_tick). A clock
//     generated in fabric would not reach a global clock buffer and
//     static timing analysis could not properly check paths in it.
//
//  2. Shift and sample always happen on OPPOSITE SCLK edges, so data
//     is stable for a full half-period before it is captured.
//
//  3. CPHA=0 samples on the FIRST edge, so the first MOSI bit must be
//     driven BEFORE that edge -- done at CS assertion (S_IDLE->S_LEAD).
//     This asymmetry is the classic SPI master bug.
//
//  4. MISO is an external asynchronous input -> two-flop synchroniser.
//     Costs 2 clk cycles of latency, so keep CLK_DIV >= 4 or the
//     sampled bit will be stale.
//
//  5. Explicit CS setup / hold states: real slaves specify t_CSS and
//     t_CSH. Going straight from CS low into clocking is a real bug.
//=====================================================================
module spi_master #(
    parameter integer DATA_WIDTH = 8,
    parameter integer CLK_DIV    = 4,     // clk cycles per SCLK HALF period (>=2)
    parameter         CPOL       = 1'b0,
    parameter         CPHA       = 1'b0,
    parameter         MSB_FIRST  = 1'b1,
    parameter integer CS_SETUP   = 4,     // clk cycles CS low before first edge
    parameter integer CS_HOLD    = 4      // clk cycles CS low after last edge
)(
    input  wire                  clk,
    input  wire                  rst_n,
    // control / data interface
    input  wire                  start,    // 1-cycle pulse, ignored when busy
    input  wire [DATA_WIDTH-1:0] tx_data,
    output reg  [DATA_WIDTH-1:0] rx_data,
    output reg                   busy,
    output reg                   done,     // 1-cycle pulse, rx_data valid
    // SPI pins
    output reg                   sclk,
    output reg                   mosi,
    output reg                   cs_n,
    input  wire                  miso
);

    localparam integer DIV_W  = (CLK_DIV < 2) ? 1 : $clog2(CLK_DIV);
    localparam integer EDGE_W = $clog2(2*DATA_WIDTH);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_LEAD  = 2'd1,   // CS setup time
                     S_XFER  = 2'd2,
                     S_TRAIL = 2'd3;   // CS hold time

    reg [1:0]            state;
    reg [DIV_W-1:0]      div_cnt;
    reg [EDGE_W:0]       edge_cnt;     // counts 0 .. 2*DATA_WIDTH-1
    reg [7:0]            cs_cnt;
    reg [DATA_WIDTH-1:0] tx_sr, rx_sr;
    reg                  miso_meta, miso_sync;

    // ---- half-period strobe: THE only thing that creates an SCLK edge ----
    wire half_tick = (div_cnt == CLK_DIV-1);

    // ---- edge classification -------------------------------------------
    // edge_cnt even  -> leading edge (first edge after CS is index 0)
    // edge_cnt odd   -> trailing edge
    wire leading     = ~edge_cnt[0];
    wire sample_edge = (CPHA == 1'b0) ?  leading : ~leading;
    wire shift_edge  = (CPHA == 1'b0) ? ~leading :  leading;

    // ---- bit-order helper ----------------------------------------------
    function [DATA_WIDTH-1:0] bitrev;
        input [DATA_WIDTH-1:0] d;
        integer i;
        begin
            for (i = 0; i < DATA_WIDTH; i = i + 1)
                bitrev[i] = d[DATA_WIDTH-1-i];
        end
    endfunction

    wire [DATA_WIDTH-1:0] tx_load = MSB_FIRST ? tx_data : bitrev(tx_data);

    // ---- two-flop synchroniser on MISO ---------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            miso_meta <= 1'b0;
            miso_sync <= 1'b0;
        end else begin
            miso_meta <= miso;
            miso_sync <= miso_meta;
        end
    end

    // ---- main FSM -------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            sclk     <= CPOL;
            cs_n     <= 1'b1;
            mosi     <= 1'b0;
            busy     <= 1'b0;
            done     <= 1'b0;
            div_cnt  <= {DIV_W{1'b0}};
            edge_cnt <= {(EDGE_W+1){1'b0}};
            cs_cnt   <= 8'd0;
            tx_sr    <= {DATA_WIDTH{1'b0}};
            rx_sr    <= {DATA_WIDTH{1'b0}};
            rx_data  <= {DATA_WIDTH{1'b0}};
        end 
        else begin
            done <= 1'b0;                       // done is a 1-cycle pulse

            case (state)

            //-------------------------------------------------------------
            S_IDLE: begin
                sclk <= CPOL;                   // hold the idle level
                if (start) begin
                    rx_sr    <= {DATA_WIDTH{1'b0}};
                    cs_n     <= 1'b0;           // assert chip select
                    busy     <= 1'b1;
                    cs_cnt   <= CS_SETUP[7:0];
                    div_cnt  <= {DIV_W{1'b0}};
                    edge_cnt <= {(EDGE_W+1){1'b0}};
                    if (CPHA == 1'b0) begin
                        // first bit must already be on MOSI at the first edge
                        mosi  <= tx_load[DATA_WIDTH-1];
                        tx_sr <= {tx_load[DATA_WIDTH-2:0], 1'b0};
                    end 
                    else begin
                        tx_sr <= tx_load;       // first bit goes out on edge 0
                    end
                    state <= S_LEAD;
                end
            end

            //-------------------------------------------------------------
            S_LEAD: begin                       // wait out CS setup time
                if (cs_cnt != 8'd0) cs_cnt <= cs_cnt - 8'd1;
                else begin
                    div_cnt <= {DIV_W{1'b0}};
                    state   <= S_XFER;
                end
            end

            //-------------------------------------------------------------
            S_XFER: begin
                if (!half_tick) begin
                    div_cnt <= div_cnt + 1'b1;
                end else begin
                    div_cnt <= {DIV_W{1'b0}};
                    sclk    <= ~sclk;           // <-- this IS the SCLK edge

                    if (sample_edge)
                        rx_sr <= {rx_sr[DATA_WIDTH-2:0], miso_sync};

                    if (shift_edge) begin
                        mosi  <= tx_sr[DATA_WIDTH-1];
                        tx_sr <= {tx_sr[DATA_WIDTH-2:0], 1'b0};
                    end

                    if (edge_cnt == 2*DATA_WIDTH-1) begin
                        edge_cnt <= {(EDGE_W+1){1'b0}};
                        cs_cnt   <= CS_HOLD[7:0];
                        state    <= S_TRAIL;
                    end else begin
                        edge_cnt <= edge_cnt + 1'b1;
                    end
                end
            end

            //-------------------------------------------------------------
            S_TRAIL: begin                      // wait out CS hold time
                sclk <= CPOL;                   // guaranteed back to idle
                if (cs_cnt != 8'd0) cs_cnt <= cs_cnt - 8'd1;
                else begin
                    cs_n    <= 1'b1;
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    rx_data <= MSB_FIRST ? rx_sr : bitrev(rx_sr);
                    state   <= S_IDLE;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
