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

    wire [31:0] haddr1, haddr2;
    wire [31:0] hwdata1, hwdata2;
    wire        hwrite_reg, hwrite_reg1;
    wire        valid;
    wire [2:0]  temp_sel;

    // Instantiate AHB Slave Interface
    ahb_slave_interface u_ahb_slave (
        .hclk(hclk),
        .hresetn(hresetn),
        .hwrite(hwrite),
        .hready_in(hready_in),
        .haddr(haddr),
        .hwdata(hwdata),
        .htrans(htrans),
        .prdata(prdata),
        .hrdata(hrdata),
        .haddr1(haddr1),
        .haddr2(haddr2),
        .hwdata1(hwdata1),
        .hwdata2(hwdata2),
        .hwrite_reg(hwrite_reg),
        .hwrite_reg1(hwrite_reg1),
        .valid(valid),
        .temp_sel(temp_sel)
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
    input  wire [31:0] hwdata,
    input  wire [1:0]  htrans,
    input  wire [31:0] prdata,
    
    output wire [31:0] hrdata,
    output reg  [31:0] haddr1,
    output reg  [31:0] haddr2,
    output reg  [31:0] hwdata1,
    output reg  [31:0] hwdata2,
    output reg         hwrite_reg,
    output reg         hwrite_reg1,
    output reg         valid,
    output reg  [2:0]  temp_sel
);

    // 1. Pipeline Logic for haddr, hwdata, hwrite
    always @(posedge hclk) begin
        if (!hresetn) begin
            haddr1      <= 0;
            haddr2      <= 0;
            hwdata1     <= 0;
            hwdata2     <= 0;
            hwrite_reg  <= 0;
            hwrite_reg1 <= 0;
        end else begin
            haddr1      <= haddr;
            haddr2      <= haddr1;
            hwdata1     <= hwdata;
            hwdata2     <= hwdata1;
            hwrite_reg  <= hwrite;
            hwrite_reg1 <= hwrite_reg;
        end
    end

    // 2. Validate Signal based on Address Range and Trans type
    always @(*) begin
        valid = 1'b0;
        if ((hready_in == 1'b1) && (haddr >= 32'h8000_0000 && haddr <= 32'h8C00_0000) && (htrans == 2'b10 || htrans == 2'b11)) begin
            valid = 1'b1;
        end
    end

    // 3. Peripheral Selection Logic
    always @(*) begin
        temp_sel = 3'b000;
        if (haddr >= 32'h8000_0000 && haddr < 32'h8400_0000)
            temp_sel = 3'b001;
        else if (haddr >= 32'h8400_0000 && haddr < 32'h8800_0000)
            temp_sel = 3'b010;
        else if (haddr >= 32'h8800_0000 && haddr < 32'h8C00_0000)
            temp_sel = 3'b100;
    end
    
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

    // Next State Logic
    always @(*) begin
        next_state = ST_IDLE;
        case(state)
            ST_IDLE: begin
                if (valid && hwrite)       next_state = ST_WWAIT;
                else if (valid && !hwrite) next_state = ST_READ;
            end
            ST_READ: next_state = ST_RENABLE;
            ST_RENABLE: begin
                if (valid && !hwrite)      next_state = ST_READ;
                else if (valid && hwrite)  next_state = ST_WWAIT;
            end     
            ST_WWAIT: begin
                if (valid)                 next_state = ST_WRITEP;
                else                       next_state = ST_WRITE;             
            end              
            ST_WRITE: begin
                if (!valid)                next_state = ST_WENABLE;
                else                       next_state = ST_WENABLEP;      
            end
            ST_WRITEP: next_state = ST_WENABLEP;        
            ST_WENABLEP: begin
                if (!hwrite_reg)                     next_state = ST_READ;
                else if (valid && hwrite_reg)        next_state = ST_WRITEP;
                else if (!valid && hwrite_reg)       next_state = ST_WRITE;
            end
            ST_WENABLE: begin
                if (valid && !hwrite)      next_state = ST_READ;
                else if (valid && hwrite)  next_state = ST_WWAIT;
            end 
        endcase
    end

    // Combinatorial Outputs
    always @(*) begin
        paddr_tmp      = paddr;
        pwdata_tmp     = pwdata;
        pwrite_tmp     = pwrite;
        psel_tmp       = psel;
        penable_tmp    = penable;
        hready_out_tmp = hready_out;

        case(state)
            ST_IDLE: begin
                if (valid && !hwrite) begin
                    paddr_tmp      = haddr;
                    pwrite_tmp     = hwrite;
                    psel_tmp       = temp_sel;
                    penable_tmp    = 0;
                    hready_out_tmp = 0;                
                end else if (valid && hwrite) begin
                    psel_tmp       = 0;
                    penable_tmp    = 0;
                    hready_out_tmp = 1;                        
                end else begin
                    psel_tmp       = 0;
                    penable_tmp    = 0;
                    hready_out_tmp = 1;                    
                end
            end
            ST_READ: begin
                psel_tmp       = temp_sel;
                penable_tmp    = 1;
                hready_out_tmp = 1;
            end
            ST_RENABLE: begin
                if (valid && !hwrite) begin
                    paddr_tmp      = haddr;
                    pwrite_tmp     = hwrite;
                    psel_tmp       = temp_sel;
                    penable_tmp    = 0;
                    hready_out_tmp = 0;                       
                end else if (valid && hwrite) begin
                    penable_tmp    = 0;
                    hready_out_tmp = 1;
                    psel_tmp       = 0;                        
                end else begin
                    psel_tmp       = 0;
                    hready_out_tmp = 1;
                    penable_tmp    = 0;
                end     
            end
            ST_WWAIT: begin 
                paddr_tmp      = haddr1;  
                pwdata_tmp     = hwdata;
                pwrite_tmp     = hwrite;
                penable_tmp    = 0;
                hready_out_tmp = 0;
                psel_tmp       = temp_sel;
            end
            ST_WRITE: begin
                penable_tmp    = 1;
                hready_out_tmp = 1;
            end   
            ST_WENABLE: begin
                if (valid && !hwrite) begin
                    hready_out_tmp = 1;
                    psel_tmp       = 0;
                    penable_tmp    = 0;
                end else if (valid && hwrite) begin
                    paddr_tmp      = haddr1;
                    penable_tmp    = 0;
                    pwrite_tmp     = hwrite;
                    psel_tmp       = temp_sel;
                    hready_out_tmp = 0;
                end else begin
                    hready_out_tmp = 1;
                    penable_tmp    = 0;
                    psel_tmp       = 0;
                end                
            end
            ST_WRITEP: begin
                hready_out_tmp = 1;
                penable_tmp    = 1;
            end              
            ST_WENABLEP: begin
                paddr_tmp      = haddr1;
                pwdata_tmp     = hwdata;
                pwrite_tmp     = hwrite;
                psel_tmp       = temp_sel;
                penable_tmp    = 0;
                hready_out_tmp = 0;
            end            
        endcase
    end    

    // Output Registers
    always @(posedge hclk) begin
        if (!hresetn) begin
            paddr      <= 0;
            pwdata     <= 0;
            penable    <= 0;
            hready_out <= 1;
            pwrite     <= 0;
            psel       <= 0;
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