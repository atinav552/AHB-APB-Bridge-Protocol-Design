// =====================================================================
// AHB-APB Bridge Testbench & Bus Functional Models (BFMs)
// =====================================================================

// ---------------------------------------------------------------------
// BFM 1: AHB Master Simulator
// ---------------------------------------------------------------------
module ahb_master_bfm (
    input  wire        hclk,
    input  wire        hresetn,
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

    task single_write;
        begin
            @(posedge hclk) begin
                #1;
                hwrite    = 1;
                htrans    = 2'b10;
                haddr     = 32'h8000_0200;
                hready_in = 1;
                hburst    = 0;
                hsize     = 0;
            end
            @(posedge hclk) begin
                #1;
                hwdata = 32'h0000_0024;
                htrans = 2'd0;    
            end
        end
    endtask                    
 
    task single_read;
        begin
            @(posedge hclk) begin
                hwrite    = 0;
                htrans    = 2'd2; // NONSEQ
                hburst    = 0;
                hsize     = 0;
                hready_in = 1;
                haddr     = 32'h8000_0100;        
            end
            @(posedge hclk) begin
                htrans = 2'd0; // IDLE       
            end    
        end
    endtask                    

    task burst_4_incr_write;
        begin
            @(posedge hclk) begin
                #1;
                hwrite    = 1;
                htrans    = 2'd2;
                hsize     = 0;
                hburst    = 0;
                hready_in = 1'b1;
                haddr     = 32'h8000_0020;    
            end
            @(posedge hclk) begin
                #1;
                haddr  = haddr + 1;
                hwdata = 32'h4940_0000;
                htrans = 2'd3; // SEQ
            end
            for (i = 0; i < 2; i = i + 1) begin
                @(posedge hclk) begin
                    #1;
                    haddr  = haddr + 1;
                    hwdata = {$random} % 256;        
                    htrans = 2'd3;
                end
            end
            @(posedge hclk) begin
                #1;
                hwdata = {$random} % 256;
                htrans = 2'd0;  
            end              
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
            prdata = 32'd25; // Dummy read data
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

    // Instantiate BFMs and DUT
    ahb_master_bfm u_ahb_master (
        .hclk(hclk),
        .hresetn(hresetn),
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

    // Clock Generation
    initial begin
        hclk = 1'b0;                                       
        forever #10 hclk = ~hclk;
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
        
        // --- Run Master Stimulus ---
        u_ahb_master.single_write();
        #20;
        u_ahb_master.single_read();
        #20;
        u_ahb_master.burst_4_incr_write();
        
        #100;
        $finish;
    end 
    
    // Waveform Dumping
    initial begin
        $dumpfile("bridge_waves.vcd");
        $dumpvars(0, ahb_apb_tb);
    end

endmodule