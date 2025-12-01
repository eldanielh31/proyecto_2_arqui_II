`timescale 1ps/1ps

// ============================================================================
// wide_onchip_mem.sv
//   - Memoria ancha de 32 bits (4 píxeles por palabra)
//   - 1 solo puerto lógico (read/write) de 32 bits
//   - Handshake simple req_valid/req_ready y resp_valid
//   - Lectura síncrona: 1 ciclo de latencia
//   - Opción CLEAR_ON_RESET:
//       * Si = 1, al salir de reset se barre toda la memoria escribiendo 0
//         (una dirección por ciclo).
// ============================================================================

module wide_onchip_mem #(
  // Para 512x512 con 4 píxeles/word: DEPTH = 65536 -> ADDR_W = 16
  parameter int ADDR_W         = 16,
  parameter bit CLEAR_ON_RESET = 1'b0
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
  // Lógica de borrado en reset (opcional)
  //   - clearing = 1 => se está barriendo la memoria a 0
  //   - clr_addr  recorre 0 .. DEPTH-1
  //   - Mientras clearing=1, req_ready=0 y se ignoran peticiones externas
  // --------------------------------------------------------------------------
  logic              clearing;
  logic [ADDR_W-1:0] clr_addr;

  // req_ready depende de si se está limpiando la RAM
  assign req_ready = (CLEAR_ON_RESET && clearing) ? 1'b0 : 1'b1;

  // --------------------------------------------------------------------------
  // Acceso síncrono con handshake
  //   - Se acepta como máximo 1 petición por ciclo (si req_ready=1).
  //   - resp_valid tiene 1 ciclo de latencia respecto a la lectura.
// ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rdata_q      <= '0;
      resp_valid_q <= 1'b0;

      if (CLEAR_ON_RESET) begin
        clearing <= 1'b1;
        clr_addr <= '0;
      end else begin
        clearing <= 1'b0;
        clr_addr <= '0;
      end

    end else begin
      // Por defecto no se afirma resp_valid en cada ciclo
      resp_valid_q <= 1'b0;

      if (CLEAR_ON_RESET && clearing) begin
        // ---------------------- Fase de borrado ----------------------
        mem[clr_addr] <= 32'd0;

        if (clr_addr == DEPTH-1) begin
          // Se termina el barrido
          clearing <= 1'b0;
        end else begin
          clr_addr <= clr_addr + 1'b1;
        end

      end else begin
        // ---------------------- Operación normal ---------------------
        if (req_valid && req_ready) begin
          // Escritura opcional
          if (req_we) begin
            mem[req_addr] <= req_wdata;
          end
          // Lectura síncrona: dato se registra
          rdata_q      <= mem[req_addr];
          resp_valid_q <= 1'b1;   // dato válido en el próximo ciclo para el lector
        end
      end
    end
  end

  assign resp_rdata = rdata_q;
  assign resp_valid = resp_valid_q;

endmodule
