`timescale 1ns/1ps

// ============================================================================
// onchip_mem_dp.sv — RAM 8-bit inferida con 4R/4W, 1 clock.
// - Síntesis: infiere M10K sin altsyncram.
// - Simulación: admite $readmemh si se define MEM_INIT_FILE y INIT_EN=1.
// ============================================================================

module onchip_mem_dp #(
  parameter int ADDR_W  = 12,
  parameter bit INIT_EN = 1'b0        // 1: cargar archivo en simulación
)(
  input  logic              clk,

  // 4 puertos de lectura
  input  logic [ADDR_W-1:0] raddr0,
  output logic [7:0]        rdata0,

  input  logic [ADDR_W-1:0] raddr1,
  output logic [7:0]        rdata1,

  input  logic [ADDR_W-1:0] raddr2,
  output logic [7:0]        rdata2,

  input  logic [ADDR_W-1:0] raddr3,
  output logic [7:0]        rdata3,

  // 4 puertos de escritura
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
  // 4R/4W, un solo reloj
  // --------------------------
  always @(posedge clk) begin
    // Escrituras (prioridad 0>1>2>3 si coinciden direcciones)
    if (we0) mem[waddr0] <= wdata0;
    if (we1) mem[waddr1] <= wdata1;
    if (we2) mem[waddr2] <= wdata2;
    if (we3) mem[waddr3] <= wdata3;

    // Lecturas síncronas
    rdata0 <= mem[raddr0];
    rdata1 <= mem[raddr1];
    rdata2 <= mem[raddr2];
    rdata3 <= mem[raddr3];
  end

endmodule
