module onchip_mem_img #(
  parameter ADDR_W = 19
)(
  input  logic              clk,
  // read port
  input  logic [ADDR_W-1:0] raddr,
  output logic [7:0]        rdata,
  // write port
  input  logic [ADDR_W-1:0] waddr,
  input  logic [7:0]        wdata,
  input  logic              we
);
  localparam DEPTH = 1<<ADDR_W;
  logic [7:0] mem [0:DEPTH-1];

  always_ff @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    rdata <= mem[raddr];
  end

  // Para simulación, permitir $readmemh desde top/tb.
endmodule
