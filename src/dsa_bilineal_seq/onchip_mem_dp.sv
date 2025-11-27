`timescale 1ps/1ps

// ============================================================================
// onchip_mem_dp.sv
//   - Memoria inferida con 4 puertos de lectura y 4 puertos de escritura.
//   - Lectura SINCRÓNICA (1 ciclo de latencia).
//   - Escrituras con prioridad de puerto 0 > 1 > 2 > 3 (si coinciden direcciones).
//   - INIT_EN:
//       * 1 -> inicializa con archivo MEM_INIT_FILE si está definido.
//       * 0 -> inicializa a 0x00.
//   - El arreglo 'mem' es visible desde el testbench: dut.mem_in.mem[addr].
// ============================================================================

module onchip_mem_dp #(
  parameter int ADDR_W  = 12,
  parameter bit INIT_EN = 1'b0
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

  // --------------------------------------------------------------------------
  // Memoria interna
  // --------------------------------------------------------------------------
  localparam int DEPTH = (1 << ADDR_W);

  // Visible desde el testbench: dut.<instancia>.mem[addr]
  logic [7:0] mem [0:DEPTH-1];

  // --------------------------------------------------------------------------
  // Inicialización (solo para simulación)
  // --------------------------------------------------------------------------
  generate
    if (INIT_EN) begin : g_init_file
      initial begin : init_with_file
      `ifdef MEM_INIT_FILE
        $display("[onchip_mem_dp] INIT_EN=1, cargando archivo HEX '%s'", `MEM_INIT_FILE);
        $readmemh(`MEM_INIT_FILE, mem);
      `else
        integer i;
        $display("[onchip_mem_dp] INIT_EN=1, pero MEM_INIT_FILE no está definido. Inicializando a 0x00.");
        for (i = 0; i < DEPTH; i = i + 1)
          mem[i] = 8'h00;
      `endif
      end
    end else begin : g_init_zero
      initial begin : init_zero
        integer i;
        $display("[onchip_mem_dp] INIT_EN=0, inicializando memoria a 0x00.");
        for (i = 0; i < DEPTH; i = i + 1)
          mem[i] = 8'h00;
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Lecturas sincronizadas y escrituras multi-puerto
  //   - rdataX se actualiza en flanco positivo (latencia 1 ciclo).
  //   - Escrituras con prioridad 0 > 1 > 2 > 3 si coinciden direcciones.
  // --------------------------------------------------------------------------
  always @(posedge clk) begin
    // Lecturas (sincrónicas)
    rdata0 <= mem[raddr0];
    rdata1 <= mem[raddr1];
    rdata2 <= mem[raddr2];
    rdata3 <= mem[raddr3];

    // Escrituras: prioridad simple por puerto
    if (we0) mem[waddr0] <= wdata0;
    if (we1) mem[waddr1] <= wdata1;
    if (we2) mem[waddr2] <= wdata2;
    if (we3) mem[waddr3] <= wdata3;
  end

endmodule
