// ============================================================================
// line_buffer.sv - NEW MODULE for caching rows
// ============================================================================
module line_buffer #(
  parameter int LINE_WIDTH = 512
)(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        wr_en,
  input  logic [9:0]  wr_addr,
  input  logic [31:0] wr_data,
  input  logic [9:0]  rd_addr,
  output logic [31:0] rd_data
);

  localparam int LINE_WORDS = (LINE_WIDTH >> 2);  // Divide by 4
  logic [31:0] buffer [0:LINE_WORDS-1];
  
  always_ff @(posedge clk) begin
    if (wr_en && wr_addr < LINE_WORDS)
      buffer[wr_addr] <= wr_data;
  end
  
  always_ff @(posedge clk) begin
    if (!rst_n)
      rd_data <= 32'h0;
    else if (rd_addr < LINE_WORDS)
      rd_data <= buffer[rd_addr];
    else
      rd_data <= 32'h0;
  end

endmodule