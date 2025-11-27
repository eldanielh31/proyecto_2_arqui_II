`timescale 1ps/1ps

// ============================================================================
// wide_onchip_mem.sv
//   - Wide memory with 32-bit words (4 pixels per word: {px3, px2, px1, px0})
//   - 2 read ports to match M10K constraints
//   - Synchronous read (1 cycle latency)
//   - 4 write ports for SIMD output
// ============================================================================
module wide_onchip_mem #(
  parameter int ADDR_W  = 10,  // Reduced by 2 bits (4 pixels per word)
  parameter bit INIT_EN = 1'b0
)(
  input  logic              clk,
  
  // 2 read ports (32-bit each = 4 pixels)
  input  logic [ADDR_W-1:0] raddr0,
  output logic [31:0]       rdata0,  // {pixel3, pixel2, pixel1, pixel0}
  
  input  logic [ADDR_W-1:0] raddr1,
  output logic [31:0]       rdata1,
  
  // 4 write ports (8-bit each)
  input  logic [ADDR_W-1:0] waddr0,
  input  logic [7:0]        wdata0,
  input  logic              we0,
  
  input  logic [ADDR_W-1:0] waddr1,
  input  logic [7:0]        wdata1,
  input  logic              we1,
  
  input  logic [ADDR_W-1:0] waddr2,
  input  logic [7:0]        wdata2,
  input  logic              we2,
  
  input  logic [ADDR_W-1:0] waddr3,
  input  logic [7:0]        wdata3,
  input  logic              we3
);

  localparam int DEPTH = (1 << ADDR_W);
  
  // Wide memory array: each entry holds 4 pixels
  logic [31:0] mem [0:DEPTH-1];
  
  // --------------------------------------------------------------------------
  // Initialization
  // --------------------------------------------------------------------------
  generate
    if (INIT_EN) begin : g_init_file
      initial begin : init_with_file
      `ifdef MEM_INIT_FILE
        $display("[wide_onchip_mem] Loading HEX file '%s'", `MEM_INIT_FILE);
        $readmemh(`MEM_INIT_FILE, mem);
      `else
        integer i;
        $display("[wide_onchip_mem] No init file, clearing to 0x00000000.");
        for (i = 0; i < DEPTH; i = i + 1)
          mem[i] = 32'h00000000;
      `endif
      end
    end else begin : g_init_zero
      initial begin : init_zero
        integer i;
        for (i = 0; i < DEPTH; i = i + 1)
          mem[i] = 32'h00000000;
      end
    end
  endgenerate
  
  // --------------------------------------------------------------------------
  // Synchronous read and write with byte-level write enable
  // --------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    // Synchronous reads
    rdata0 <= mem[raddr0];
    rdata1 <= mem[raddr1];
    
    // Byte-granular writes (priority 0 > 1 > 2 > 3)
    // Each write port can update one byte within a 32-bit word
    if (we0) begin
      case (waddr0[1:0])  // Lower 2 bits determine byte position
        2'b00: mem[waddr0[ADDR_W-1:2]][7:0]   <= wdata0;
        2'b01: mem[waddr0[ADDR_W-1:2]][15:8]  <= wdata0;
        2'b10: mem[waddr0[ADDR_W-1:2]][23:16] <= wdata0;
        2'b11: mem[waddr0[ADDR_W-1:2]][31:24] <= wdata0;
      endcase
    end
    
    if (we1) begin
      case (waddr1[1:0])
        2'b00: mem[waddr1[ADDR_W-1:2]][7:0]   <= wdata1;
        2'b01: mem[waddr1[ADDR_W-1:2]][15:8]  <= wdata1;
        2'b10: mem[waddr1[ADDR_W-1:2]][23:16] <= wdata1;
        2'b11: mem[waddr1[ADDR_W-1:2]][31:24] <= wdata1;
      endcase
    end
    
    if (we2) begin
      case (waddr2[1:0])
        2'b00: mem[waddr2[ADDR_W-1:2]][7:0]   <= wdata2;
        2'b01: mem[waddr2[ADDR_W-1:2]][15:8]  <= wdata2;
        2'b10: mem[waddr2[ADDR_W-1:2]][23:16] <= wdata2;
        2'b11: mem[waddr2[ADDR_W-1:2]][31:24] <= wdata2;
      endcase
    end
    
    if (we3) begin
      case (waddr3[1:0])
        2'b00: mem[waddr3[ADDR_W-1:2]][7:0]   <= wdata3;
        2'b01: mem[waddr3[ADDR_W-1:2]][15:8]  <= wdata3;
        2'b10: mem[waddr3[ADDR_W-1:2]][23:16] <= wdata3;
        2'b11: mem[waddr3[ADDR_W-1:2]][31:24] <= wdata3;
      endcase
    end
  end

endmodule

