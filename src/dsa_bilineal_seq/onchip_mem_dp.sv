// onchip_mem_dp.sv — RAM 8-bit inferida (simple dual-port: 1R/1W, 1 clk)
// - Síntesis: infiere M10K sin dependencias de altsyncram.
// - Simulación: admite $readmemh si se define MEM_INIT_FILE y INIT_EN=1.

`timescale 1ns/1ps

module onchip_mem_dp #(
  parameter int ADDR_W  = 12,
  parameter bit INIT_EN = 1'b0        // 1: cargar archivo en simulación
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

  localparam int DEPTH = 1 << ADDR_W;

  // M10K inferido
  (* ramstyle = "M10K" *) logic [7:0] mem [0:DEPTH-1];

  // --------------------------
  // Simulación: carga opcional
  // --------------------------
  // synthesis translate_off
  integer i;
  initial begin
    for (i = 0; i < DEPTH; i++) mem[i] = 8'h00;
`ifdef MEM_INIT_FILE
    if (INIT_EN) begin
      $display("onchip_mem_dp (sim): $readmemh from %s", `MEM_INIT_FILE);
      $readmemh(`MEM_INIT_FILE, mem);
    end
`endif
  end
  // synthesis translate_on

  // --------------------------
  // 1R/1W, un solo reloj
  // --------------------------
  // NOTA: usamos 'always' en lugar de 'always_ff' para evitar el conflicto
  // con el 'initial' que también escribe en 'mem' durante simulación.
  always @(posedge clk) begin
    if (we) begin
      mem[waddr] <= wdata;
    end
    // lectura síncrona
    rdata <= mem[raddr];
  end

endmodule
