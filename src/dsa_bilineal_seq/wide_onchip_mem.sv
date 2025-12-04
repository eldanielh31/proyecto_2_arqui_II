`timescale 1ps/1ps

// ============================================================================
// wide_onchip_mem.sv (versión optimizada)
//   - Memoria ancha de 32 bits (4 píxeles por palabra)
//   - 1 solo puerto lógico (read/write) de 32 bits
//   - Handshake simple req_valid/req_ready y resp_valid
//   - Lectura síncrona: 1 ciclo de latencia
// ============================================================================

module wide_onchip_mem #(
  // Para 512x512 con 4 píxeles/word: DEPTH = 65536 -> ADDR_W = 16
  parameter int ADDR_W  = 18,
  parameter bit INIT_EN = 1'b0
)(
  input  logic              clk,
  input  logic              rst_n,

  // Canal de petición (read/write)
  input  logic              req_valid,   // petición válida (read o write)
  output logic              req_ready,   // memoria puede aceptar petición
  input  logic              req_we,      // 1 = write, 0 = read
  input  logic [ADDR_W-1:0] req_addr,    // dirección de palabra (32 bits)
  input  logic [31:0]       req_wdata,   // dato a escribir (si req_we=1)

  // Canal de respuesta (lectura)
  output logic              resp_valid,  // dato de lectura válido
  output logic [31:0]       resp_rdata   // palabra leída
);

  localparam int DEPTH = (1 << ADDR_W);

  // Memoria: cada entrada = 4 píxeles de 8 bits
  logic [31:0] mem [0:DEPTH-1];

  logic [31:0] rdata_q;
  logic        resp_valid_q;

  // --------------------------------------------------------------------------
  // Inicialización (opcional con archivo HEX)
  // --------------------------------------------------------------------------
//  generate
//    if (INIT_EN) begin : g_init_file
//      initial begin
//      `ifdef MEM_INIT_FILE
//        $display("[wide_onchip_mem] Loading HEX file '%s'", `MEM_INIT_FILE);
//        $readmemh(`MEM_INIT_FILE, mem);
//      `else
//        integer i;
//        $display("[wide_onchip_mem] No init file, clearing to 0x00000000.");
//        for (i = 0; i < DEPTH; i = i + 1)
//          mem[i] = 32'h00000000;
//      `endif
//      end
//    end else begin : g_init_zero
//      initial begin
//        integer i;
//        for (i = 0; i < DEPTH; i = i + 1)
//          mem[i] = 32'h00000000;
//      end
//    end
//  endgenerate

  // --------------------------------------------------------------------------
  // Acceso síncrono con handshake
  //   - Se acepta como máximo 1 petición por ciclo (req_ready=1 siempre).
  //   - resp_valid es req_valid retrasado 1 ciclo.
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rdata_q      <= '0;
      resp_valid_q <= 1'b0;
    end else begin
      if (req_valid) begin
        // Escritura opcional
        if (req_we) begin
          mem[req_addr] <= req_wdata;
        end
        // Lectura síncrona: dato se registra
        rdata_q <= mem[req_addr];
      end

      // resp_valid va alineado 1 ciclo después de req_valid
      resp_valid_q <= req_valid;
    end
  end

  assign resp_rdata = rdata_q;
  assign resp_valid = resp_valid_q;

  // La RAM siempre puede aceptar una petición por ciclo
  assign req_ready  = 1'b1;

endmodule
