// ============================================================================
// tb_read_engine.sv  -- self-checking unit testbench for read_engine
// ----------------------------------------------------------------------------
// The tb plays BOTH ends: upstream (issues line-fetch requests, checks the
// returned line) and a behavioural Ld_* memory (accepts requests, returns
// data = a deterministic function of the address). Verifies the full
// req -> Ld request -> Ld data -> rsp path. Plain SV-2012. Design: read_engine.sv.
// ============================================================================
`timescale 1ns/1ps
module tb_read_engine;

    localparam int SA_SIZE   = 4;
    localparam int LD_ADDR_W = 32;
    localparam int WCOUNT_W  = 6;
    localparam int LINE_W    = SA_SIZE*32;   // 128

    logic clk=0, rst_n;
    logic                 rd_req_valid, rd_req_ready;
    logic [LD_ADDR_W-1:0] rd_req_addr;
    logic                 rd_rsp_valid, rd_rsp_ready;
    logic [LINE_W-1:0]    rd_rsp_data;
    logic                 ld_req_valid, ld_req_ready;
    logic [LD_ADDR_W-1:0] ld_addr;
    logic [WCOUNT_W-1:0]  ld_wcount;
    logic                 ld_rnw;
    logic                 ld_dat_valid, ld_dat_ready;
    logic [LINE_W-1:0]    ld_dat;

    integer errors=0, tests=0;

    read_engine #(.SA_SIZE(SA_SIZE), .LD_ADDR_W(LD_ADDR_W), .WCOUNT_W(WCOUNT_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .rd_req_valid(rd_req_valid), .rd_req_addr(rd_req_addr), .rd_req_ready(rd_req_ready),
        .rd_rsp_valid(rd_rsp_valid), .rd_rsp_data(rd_rsp_data), .rd_rsp_ready(rd_rsp_ready),
        .ld_req_valid(ld_req_valid), .ld_req_ready(ld_req_ready),
        .ld_addr(ld_addr), .ld_wcount(ld_wcount), .ld_rnw(ld_rnw),
        .ld_dat_valid(ld_dat_valid), .ld_dat_ready(ld_dat_ready), .ld_dat(ld_dat)
    );

    always #5 clk = ~clk;

    // ---- behavioural Ld_* memory ----------------------------------------
    logic [LD_ADDR_W-1:0] mem_addr_q;
    logic                 mem_has;
    assign ld_req_ready = 1'b1;                       // always accept a request
    assign ld_dat_valid = mem_has;
    assign ld_dat = { (mem_addr_q+3), (mem_addr_q+2), (mem_addr_q+1), mem_addr_q }; // 4x32

    function [LINE_W-1:0] expected(input [LD_ADDR_W-1:0] a);
        expected = { (a+3), (a+2), (a+1), a };
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin mem_has<=1'b0; mem_addr_q<='0; end
        else begin
            if (ld_req_valid && ld_req_ready) begin mem_addr_q <= ld_addr; mem_has <= 1'b1; end
            else if (ld_dat_valid && ld_dat_ready) mem_has <= 1'b0;
        end
    end

    // ---- upstream: one read, then check ---------------------------------
    task do_read(input [LD_ADDR_W-1:0] addr);
        begin
            @(negedge clk); rd_req_valid=1; rd_req_addr=addr;
            // wait until the request is accepted (sample at the edge)
            @(posedge clk);
            while (!rd_req_ready) @(posedge clk);
            @(negedge clk); rd_req_valid=0;
            // wait for the response and check it
            rd_rsp_ready=1;
            @(posedge clk);
            while (!rd_rsp_valid) @(posedge clk);
            tests = tests + 1;
            if (rd_rsp_data !== expected(addr)) begin
                $display("  FAIL: addr=0x%0h -> data=0x%032h (exp 0x%032h)",
                         addr, rd_rsp_data, expected(addr));
                errors = errors + 1;
            end else
                $display("  ok  : addr=0x%0h -> line correct (wcount=%0d rnw=%b)",
                         addr, ld_wcount, ld_rnw);
            @(negedge clk); rd_rsp_ready=0;
        end
    endtask

    initial begin
        rst_n=0; rd_req_valid=0; rd_req_addr=0; rd_rsp_ready=0;
        repeat(3) @(negedge clk); rst_n=1;

        $display("[tb_read_engine] starting");
        do_read(32'h0000_0000);
        do_read(32'h0000_1000);
        do_read(32'h1234_5670);
        do_read(32'hDEAD_BEE0);
        do_read(32'h0000_0010);

        if (errors==0) $display("[tb_read_engine] PASS (%0d reads)", tests);
        else           $display("[tb_read_engine] FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin #200000; $display("[tb_read_engine] TIMEOUT"); $finish; end
endmodule
