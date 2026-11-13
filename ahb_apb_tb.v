`timescale 1ns/1ps

// =====================================================================
// AHB-APB Bridge Testbench & Bus Functional Models (BFMs)
// =====================================================================

// ---------------------------------------------------------------------
// BFM 1: AHB Master Simulator (With hready_out Stall Support)
// ---------------------------------------------------------------------
module ahb_master_bfm (
    input  wire        hclk,
    input  wire        hresetn,
    input  wire        hready_out, // Added to respect bridge wait states
    input  wire [31:0] hrdata,
    output reg         hwrite,
    output reg         hready_in,
    output reg  [1:0]  htrans,
    output reg  [31:0] hwdata,
    output reg  [31:0] haddr
);
                    
    reg [2:0] hburst;
    reg [2:0] hsize;
    integer i;

    initial begin
        hwrite    = 1'b0;
        hready_in = 1'b1;
        htrans    = 2'b00;
        hwdata    = 32'd0;
        haddr     = 32'd0;
        hburst    = 3'b000;
        hsize     = 3'b010;
    end

    task single_write;
        begin
            // Address Phase
            @(posedge hclk);
            #1;
            hwrite    = 1'b1;
            htrans    = 2'b10; // NONSEQ
            haddr     = 32'h8000_0200;
            hready_in = 1'b1;
            hburst    = 3'b000;
            hsize     = 3'b010; // 32-bit Word

            // Data Phase
            @(posedge hclk);
            #1;
            hwdata    = 32'h0000_0024;
            htrans    = 2'b00; // IDLE
        end
    endtask                    
 
    task single_read;
        begin
            // Address Phase
            @(posedge hclk);
            #1; // Fixed race condition
            hwrite    = 1'b0;
            htrans    = 2'b10; // NONSEQ
            hburst    = 3'b000;
            hsize     = 3'b010;
            hready_in = 1'b1;
            haddr     = 32'h8000_0100;        

            // Data Phase (Wait for bridge to complete APB read)
            @(posedge hclk);
            #1;
            htrans    = 2'b00; // IDLE        
        end
    endtask                    

    task burst_4_incr_write;
        begin
            // Beat 1: Address Phase
            @(posedge hclk);
            #1;
            hwrite    = 1'b1;
            htrans    = 2'b10; // NONSEQ
            hsize     = 3'b010; // 32-bit Word
            hburst    = 3'b011; // INCR4
            hready_in = 1'b1;
            haddr     = 32'h8000_0020;    

            // Beat 1 Data Phase + Beat 2 Address Phase
            @(posedge hclk);
            #1;
            haddr     = haddr + 32'd4;
            hwdata    = 32'h4940_0001;
            htrans    = 2'b11; // SEQ

            // Beats 3 & 4: Respect hready_out wait states from APB Bridge
            for (i = 0; i < 2; i = i + 1) begin
                @(posedge hclk);
                while (!hready_out) @(posedge hclk); // Stall master while APB is busy
                #1;
                haddr  = haddr + 32'd4;
                hwdata = 32'h4940_0002 + i;        
                htrans = 2'b11; // SEQ
            end

            // Final Data Phase (Beat 4 Data)
            @(posedge hclk);
            while (!hready_out) @(posedge hclk);
            #1;
            hwdata = 32'h4940_0004;
            htrans = 2'b00; // IDLE  
        end 
    endtask                    
endmodule

// ---------------------------------------------------------------------
// BFM 2: APB Peripheral Slave Simulator
// ---------------------------------------------------------------------
module apb_slave_bfm (
    input  wire        pwrite,
    input  wire        penable,
    input  wire [2:0]  psel,
    input  wire [31:0] paddr,
    input  wire [31:0] pwdata,
    
    output wire        pwrite_out,
    output wire        penable_out,
    output wire [2:0]  psel_out,
    output wire [31:0] paddr_out,
    output wire [31:0] pwdata_out,
    output reg  [31:0] prdata
);
                        
    assign pwrite_out  = pwrite;
    assign paddr_out   = paddr;
    assign psel_out    = psel;
    assign pwdata_out  = pwdata;
    assign penable_out = penable; 

    always @(*) begin
        if (!pwrite && penable) begin
            prdata = 32'd25; // Dummy read data (0x0000_0019)
        end else begin
            prdata = 32'd0;
        end
    end                        
endmodule

// ---------------------------------------------------------------------
// Testbench Top
// ---------------------------------------------------------------------
module ahb_apb_tb;

    reg         hclk, hresetn;
    wire [31:0] hrdata, haddr, hwdata, prdata;
    wire        hwrite, hready_in;
    wire [1:0]  htrans;

    wire        penable, pwrite, hready_out;
    wire [2:0]  psel;
    wire [31:0] paddr, pwdata;

    wire        pwrite_out, penable_out;
    wire [2:0]  psel_out;
    wire [31:0] paddr_out, pwdata_out;

    integer pass_cnt = 0;

    // Instantiate BFMs and DUT
    ahb_master_bfm u_ahb_master (
        .hclk(hclk),
        .hresetn(hresetn),
        .hready_out(hready_out),
        .hrdata(hrdata),
        .hwrite(hwrite),
        .hready_in(hready_in),
        .htrans(htrans),
        .hwdata(hwdata),
        .haddr(haddr)
    );
                   
    ahb_apb_bridge u_dut_bridge ( 
        .hclk(hclk),
        .hresetn(hresetn),
        .hwrite(hwrite),
        .hready_in(hready_in),
        .htrans(htrans),
        .haddr(haddr),
        .hwdata(hwdata),
        .prdata(prdata),
        .penable(penable), 
        .pwrite(pwrite),
        .hready_out(hready_out),
        .psel(psel),
        .paddr(paddr), 
        .pwdata(pwdata), 
        .hrdata(hrdata)
    );               

    apb_slave_bfm u_apb_slave (
        .pwrite(pwrite),
        .penable(penable),
        .psel(psel),
        .paddr(paddr),
        .pwdata(pwdata),
        .pwrite_out(pwrite_out),
        .penable_out(penable_out),
        .psel_out(psel_out),
        .paddr_out(paddr_out),
        .pwdata_out(pwdata_out),
        .prdata(prdata)
    );

    // Clock Generation (20ns period / 50MHz)
    initial begin
        hclk = 1'b0;                                                    
        forever #10 hclk = ~hclk;
    end
 
    // Self-Checking APB Protocol Monitor
    always @(posedge hclk) begin
        if (hresetn && penable && (|psel)) begin
            if (pwrite) begin
                $display("[APB WRITE COMPLETE] Time=%0t | PSEL=%b | PADDR=%h | PWDATA=%h", 
                         $time, psel, paddr, pwdata);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("[APB READ COMPLETE]  Time=%0t | PSEL=%b | PADDR=%h | HRDATA=%0d", 
                         $time, psel, paddr, hrdata);
                pass_cnt = pass_cnt + 1;
            end
        end
    end

    task reset_sys;
        begin
            @(negedge hclk);
            hresetn = 1'b0;
            @(negedge hclk);
            hresetn = 1'b1;
        end
    endtask         

    initial begin
        reset_sys();
        
        // 1. Single Write Test (Wait 60ns after so ST_WRITE -> ST_WENABLE completes)
        u_ahb_master.single_write();
        #60;
        
        // 2. Single Read Test
        u_ahb_master.single_read();
        #60;
        
        // 3. 4-Beat Pipelined Burst Write Test
        u_ahb_master.burst_4_incr_write();
        
        #120;
        $display("==========================================================");
        $display("TOTAL APB TRANSACTIONS VERIFIED: %0d (Expected: 6)", pass_cnt);
        $display("==========================================================");
        $finish;
    end 
    
    // Waveform Dumping
    initial begin
        $dumpfile("bridge_waves.vcd");
        $dumpvars(0, ahb_apb_tb);
    end

endmodule
