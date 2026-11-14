// =====================================================================
// AHB to APB Bridge Protocol
// =====================================================================

// ---------------------------------------------------------------------
// Module 1: Top Level Wrapper
// ---------------------------------------------------------------------
module ahb_apb_bridge (
    input  wire        hclk,
    input  wire        hresetn,
    input  wire        hwrite,
    input  wire        hready_in,
    input  wire [1:0]  htrans,
    input  wire [31:0] haddr,
    input  wire [31:0] hwdata,
    input  wire [31:0] prdata,
    
    output wire        penable,
    output wire        pwrite,
    output wire        hready_out,
    output wire [2:0]  psel,
    output wire [31:0] paddr,
    output wire [31:0] pwdata,
    output wire [31:0] hrdata
);

    wire [31:0] haddr1;
    wire        hwrite_reg;
    wire        valid;
    wire [2:0]  temp_sel, temp_sel_reg;

    // Instantiate AHB Slave Interface
    ahb_slave_interface u_ahb_slave (
        .hclk(hclk),
        .hresetn(hresetn),
        .hwrite(hwrite),
        .hready_in(hready_in),
        .haddr(haddr),
        .htrans(htrans),
        .prdata(prdata),
        .hrdata(hrdata),
        .haddr1(haddr1),
        .hwrite_reg(hwrite_reg),
        .valid(valid),
        .temp_sel(temp_sel),
        .temp_sel_reg(temp_sel_reg)
    );

    // Instantiate APB Controller FSM
    apb_fsm_controller u_apb_ctrl (
        .hclk(hclk),
        .hresetn(hresetn),
        .hwrite_reg(hwrite_reg),
        .hwrite(hwrite),
        .valid(valid),
        .haddr(haddr),
        .haddr1(haddr1),
        .hwdata(hwdata),
        .temp_sel(temp_sel),
        .temp_sel_reg(temp_sel_reg),
        .penable(penable),
        .pwrite(pwrite),
        .hready_out(hready_out),
        .psel(psel),
        .paddr(paddr),
        .pwdata(pwdata)
    );

endmodule

// ---------------------------------------------------------------------
// Module 2: AHB Slave Interface
// ---------------------------------------------------------------------
module ahb_slave_interface (
    input  wire        hclk,
    input  wire        hresetn,
    input  wire        hwrite,
    input  wire        hready_in,
    input  wire [31:0] haddr,
    input  wire [1:0]  htrans,
    input  wire [31:0] prdata,
    
    output wire [31:0] hrdata,
    output reg  [31:0] haddr1,
    output reg         hwrite_reg,
    output wire        valid,
    output reg  [2:0]  temp_sel,
    output reg  [2:0]  temp_sel_reg
);

    // 1. 1-Stage Pipeline Logic for haddr, hwrite, and temp_sel
    always @(posedge hclk) begin
        if (!hresetn) begin
            haddr1       <= 32'd0;
            hwrite_reg   <= 1'b0;
            temp_sel_reg <= 3'b000;
        end else begin
            haddr1       <= haddr;
            hwrite_reg   <= hwrite;
            temp_sel_reg <= temp_sel;
        end
    end

    // 2. Peripheral Selection Logic
    always @(*) begin
        temp_sel = 3'b000;
        if (haddr >= 32'h8000_0000 && haddr < 32'h8400_0000)
            temp_sel = 3'b001;
        else if (haddr >= 32'h8400_0000 && haddr < 32'h8800_0000)
            temp_sel = 3'b010;
        else if (haddr >= 32'h8800_0000 && haddr < 32'h8C00_0000)
            temp_sel = 3'b100;
    end

    // 3. Validate Signal (Reuses temp_sel range check & checks htrans[1] for NONSEQ/SEQ)
    assign valid  = hready_in && (|temp_sel) && htrans[1];
    assign hrdata = prdata;

endmodule

// ---------------------------------------------------------------------
// Module 3: APB FSM Controller
// ---------------------------------------------------------------------
module apb_fsm_controller (
    input  wire        hclk, 
    input  wire        hresetn, 
    input  wire        hwrite_reg, 
    input  wire        hwrite, 
    input  wire        valid,
    input  wire [31:0] haddr, 
    input  wire [31:0] haddr1, 
    input  wire [31:0] hwdata,
    input  wire [2:0]  temp_sel,
    input  wire [2:0]  temp_sel_reg,
    
    output reg         penable, 
    output reg         pwrite,
    output reg         hready_out,
    output reg  [2:0]  psel,
    output reg  [31:0] paddr, 
    output reg  [31:0] pwdata
);

    parameter ST_IDLE     = 3'b000,
              ST_READ     = 3'b001,
              ST_RENABLE  = 3'b010,
              ST_WENABLE  = 3'b011,
              ST_WRITE    = 3'b100,
              ST_WWAIT    = 3'b101,
              ST_WRITEP   = 3'b110,
              ST_WENABLEP = 3'b111;                        
                        
    reg [2:0] state, next_state;

    // Temporary combinatorial routing
    reg        penable_tmp, pwrite_tmp, hready_out_tmp;
    reg [2:0]  psel_tmp;
    reg [31:0] paddr_tmp, pwdata_tmp;

    // State Register
    always @(posedge hclk) begin
        if (!hresetn) state <= ST_IDLE;
        else          state <= next_state;
    end

    // Next State Logic (Grouped & Simplified)
    always @(*) begin
        next_state = ST_IDLE;
        case(state)
            ST_IDLE, ST_RENABLE, ST_WENABLE: begin
                if (valid) next_state = hwrite ? ST_WWAIT : ST_READ;
            end
            ST_READ:   next_state = ST_RENABLE;
            ST_WWAIT:  next_state = valid ? ST_WRITEP   : ST_WRITE;
            ST_WRITE:  next_state = valid ? ST_WENABLEP : ST_WENABLE;
            ST_WRITEP: next_state = ST_WENABLEP;        
            ST_WENABLEP: begin
                if (!hwrite_reg) next_state = ST_READ;
                else if (valid)  next_state = ST_WRITEP;
                else             next_state = ST_WRITE;
            end
        endcase
    end

    // Combinatorial Look-Ahead Outputs (Merged Duplicate Cases)
    always @(*) begin
        paddr_tmp      = paddr;
        pwdata_tmp     = pwdata;
        pwrite_tmp     = pwrite;
        psel_tmp       = psel;
        penable_tmp    = penable;
        hready_out_tmp = hready_out;

        case(state)
            ST_IDLE, ST_RENABLE, ST_WENABLE: begin
                if (valid && !hwrite) begin
                    // Transitioning to ST_READ (APB Read Setup)
                    paddr_tmp      = haddr;
                    pwrite_tmp     = 1'b0;
                    psel_tmp       = temp_sel;
                    penable_tmp    = 1'b0;
                    hready_out_tmp = 1'b0;                
                end else begin
                    // Transitioning to ST_WWAIT or returning to ST_IDLE
                    psel_tmp       = 3'b000;
                    penable_tmp    = 1'b0;
                    hready_out_tmp = 1'b1;                    
                end
            end

            ST_READ, ST_WRITE, ST_WRITEP: begin
                // Transitioning to Enable Phase (ST_RENABLE, ST_WENABLE, or ST_WENABLEP)
                penable_tmp    = 1'b1;
                hready_out_tmp = 1'b1;
            end

            ST_WWAIT, ST_WENABLEP: begin 
                // Transitioning to Setup Phase using pipelined address (haddr1)
                paddr_tmp      = haddr1;  
                pwdata_tmp     = hwdata;
                pwrite_tmp     = hwrite_reg;
                psel_tmp       = temp_sel_reg;
                penable_tmp    = 1'b0;
                hready_out_tmp = 1'b0;
            end
        endcase
    end    

    // Output Registers
    always @(posedge hclk) begin
        if (!hresetn) begin
            paddr      <= 32'd0;
            pwdata     <= 32'd0;
            penable    <= 1'b0;
            hready_out <= 1'b1;
            pwrite     <= 1'b0;
            psel       <= 3'b000;
        end else begin
            paddr      <= paddr_tmp;
            penable    <= penable_tmp;
            psel       <= psel_tmp;
            pwdata     <= pwdata_tmp;
            hready_out <= hready_out_tmp;
            pwrite     <= pwrite_tmp;        
        end
    end 
endmodule
