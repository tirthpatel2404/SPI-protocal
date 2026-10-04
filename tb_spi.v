`timescale 1ns/1ps
//=====================================================================
// Behavioural SPI slave model -- TESTBENCH ONLY, not synthesisable.
// It clocks off SCLK directly, which is legal for a model and illegal
// for the design under test.
//=====================================================================
module spi_slave_model #(
    parameter integer DATA_WIDTH = 8,
    parameter         CPOL       = 1'b0,
    parameter         CPHA       = 1'b0
)(
    input  wire                  cs_n,
    input  wire                  sclk,
    input  wire                  mosi,
    output reg                   miso,
    input  wire [DATA_WIDTH-1:0] tx_data,
    output reg  [DATA_WIDTH-1:0] rx_data
);
    reg [DATA_WIDTH-1:0] tx_sr, rx_sr;

    initial begin miso = 1'b0; tx_sr = 0; rx_sr = 0; rx_data = 0; end

    always @(negedge cs_n) begin
        rx_sr <= {DATA_WIDTH{1'b0}};
        if (CPHA == 1'b0) begin
            miso  <= tx_data[DATA_WIDTH-1];               // present first bit
            tx_sr <= {tx_data[DATA_WIDTH-2:0], 1'b0};
        end else begin
            tx_sr <= tx_data;
        end
    end

    task do_sample; begin rx_sr <= {rx_sr[DATA_WIDTH-2:0], mosi}; end endtask
    task do_shift;  begin
        miso  <= tx_sr[DATA_WIDTH-1];
        tx_sr <= {tx_sr[DATA_WIDTH-2:0], 1'b0};
    end endtask

    // CPOL=0 : posedge is the LEADING edge.  CPOL=1 : negedge is leading.
    always @(posedge sclk) if (!cs_n) begin
        if (CPOL == 1'b0) begin                 // leading
            if (CPHA == 1'b0) do_sample; else do_shift;
        end else begin                          // trailing
            if (CPHA == 1'b0) do_shift;  else do_sample;
        end
    end

    always @(negedge sclk) if (!cs_n) begin
        if (CPOL == 1'b1) begin                 // leading
            if (CPHA == 1'b0) do_sample; else do_shift;
        end else begin                          // trailing
            if (CPHA == 1'b0) do_shift;  else do_sample;
        end
    end

    always @(posedge cs_n) rx_data <= rx_sr;
endmodule

//=====================================================================
// One master+slave pair for a given mode, self-checking.
//=====================================================================
module tb_pair #(
    parameter CPOL = 1'b0,
    parameter CPHA = 1'b0,
    parameter MSB_FIRST = 1'b1
)(input wire clk, input wire rst_n, output reg [15:0] errors, output reg done_flag);

    localparam DW = 8;

    reg  [DW-1:0] m_tx, s_tx;
    reg           start;
    wire [DW-1:0] m_rx, s_rx;
    wire          busy, xdone, sclk, mosi, cs_n, miso;

    spi_master #(.DATA_WIDTH(DW), .CLK_DIV(4), .CPOL(CPOL), .CPHA(CPHA),
                 .MSB_FIRST(MSB_FIRST), .CS_SETUP(4), .CS_HOLD(4))
      dut (.clk(clk), .rst_n(rst_n), .start(start), .tx_data(m_tx),
           .rx_data(m_rx), .busy(busy), .done(xdone),
           .sclk(sclk), .mosi(mosi), .cs_n(cs_n), .miso(miso));

    spi_slave_model #(.DATA_WIDTH(DW), .CPOL(CPOL), .CPHA(CPHA))
      slv (.cs_n(cs_n), .sclk(sclk), .mosi(mosi), .miso(miso),
           .tx_data(s_tx), .rx_data(s_rx));

    integer i;
    reg [DW-1:0] vec [0:5];
    reg [DW-1:0] exp_m, exp_s;

    // the slave model is always MSB-first, so when the master is LSB-first
    // the bytes appear bit-reversed on both sides.
    function [DW-1:0] rev; input [DW-1:0] d; integer k;
        begin for (k=0;k<DW;k=k+1) rev[k] = d[DW-1-k]; end
    endfunction

    initial begin
        errors = 0; done_flag = 0; start = 0; m_tx = 0; s_tx = 0;
        vec[0]=8'hA5; vec[1]=8'h00; vec[2]=8'hFF; vec[3]=8'h01;
        vec[4]=8'h80; vec[5]=8'h3C;

        @(posedge rst_n); repeat (5) @(posedge clk);

        for (i = 0; i < 6; i = i + 1) begin
            m_tx = vec[i];
            s_tx = ~vec[i];
            @(posedge clk); start = 1;
            @(posedge clk); start = 0;
            @(posedge xdone);
            @(posedge clk);
            exp_m = MSB_FIRST ? (~vec[i]) : rev(~vec[i]);
            exp_s = MSB_FIRST ?   vec[i]  : rev(  vec[i]);
            if (m_rx !== exp_m) begin
                errors = errors + 1;
                $display("  FAIL master_rx=%02h expected=%02h", m_rx, exp_m);
            end
            if (s_rx !== exp_s) begin
                errors = errors + 1;
                $display("  FAIL slave_rx =%02h expected=%02h", s_rx, exp_s);
            end
            repeat (4) @(posedge clk);
        end
        done_flag = 1;
    end
endmodule

//=====================================================================
module tb_spi;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;                       // 100 MHz

    wire [15:0] e0,e1,e2,e3,e4;
    wire d0,d1,d2,d3,d4;

    tb_pair #(.CPOL(1'b0), .CPHA(1'b0)) m0 (clk, rst_n, e0, d0);
    tb_pair #(.CPOL(1'b0), .CPHA(1'b1)) m1 (clk, rst_n, e1, d1);
    tb_pair #(.CPOL(1'b1), .CPHA(1'b0)) m2 (clk, rst_n, e2, d2);
    tb_pair #(.CPOL(1'b1), .CPHA(1'b1)) m3 (clk, rst_n, e3, d3);
    // LSB-first spot check in mode 0
    tb_pair #(.CPOL(1'b0), .CPHA(1'b0), .MSB_FIRST(1'b0)) m4 (clk, rst_n, e4, d4);

    initial begin
        $dumpfile("spi.vcd"); $dumpvars(0, tb_spi);
        repeat (10) @(posedge clk);
        rst_n = 1;
        wait (d0 && d1 && d2 && d3 && d4);
        $display("--------------------------------------------");
        $display("mode0 (CPOL=0,CPHA=0) errors = %0d", e0);
        $display("mode1 (CPOL=0,CPHA=1) errors = %0d", e1);
        $display("mode2 (CPOL=1,CPHA=0) errors = %0d", e2);
        $display("mode3 (CPOL=1,CPHA=1) errors = %0d", e3);
        $display("mode0 LSB-first       errors = %0d", e4);
        $display("--------------------------------------------");
        if ((e0|e1|e2|e3|e4) == 0) $display("*** ALL MODES PASS ***");
        else                       $display("*** FAILURES ***");
        $finish;
    end

    initial begin #500000; $display("TIMEOUT"); $finish; end
endmodule
