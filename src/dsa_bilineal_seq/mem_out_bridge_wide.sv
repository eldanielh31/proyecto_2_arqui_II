`timescale 1ps/1ps

// ============================================================================
// mem_out_bridge_wide.sv
//   - Puente entre los cores (seq/SIMD4) y la memoria de salida wide (32 bits)
//   - Funciones:
//       * Multiplexa las escrituras del core activo (seq / simd)
//       * Cola las escrituras en un FIFO para desacoplar vs. latencia de mem_out
//       * Genera las peticiones req_valid/req_we/addr/wdata hacia mem_out
//       * Atiende lecturas desde JTAG como bytes (vista 8 bits) usando la
//         misma memoria wide de 32 bits, vía FSM de lectura
//       * Árbitro simple: prioridad a escrituras del core, luego lecturas JTAG
// ============================================================================

module mem_out_bridge_wide #(
  parameter int AW             = 16,
  parameter int OUT_FIFO_DEPTH = 32
)(
  input  logic        clk,
  input  logic        rst_n,

  // Modo SIMD (para seleccionar core)
  input  logic        mode_simd_eff,

  // ---------------------- Puertos desde core secuencial ---------------------
  input  logic [AW-1:0] out_mem_addr_seq,
  input  logic [31:0]   out_mem_wdata_seq,
  input  logic          out_mem_we_seq,

  // ------------------------ Puertos desde core SIMD4 ------------------------
  input  logic [AW-1:0] out_mem_addr_simd,
  input  logic [31:0]   out_mem_wdata_simd,
  input  logic          out_mem_we_simd,

  // -------------------- Interfaz hacia mem_out (wide RAM) -------------------
  output logic              mem_out_req_valid,
  input  logic              mem_out_req_ready,
  output logic              mem_out_req_we,
  output logic [AW-1:0]     mem_out_req_addr,
  output logic [31:0]       mem_out_req_wdata,
  input  logic              mem_out_resp_valid,
  input  logic [31:0]       mem_out_resp_rdata,

  // ------------------- Interfaz hacia JTAG (vista 8 bits) -------------------
  input  logic [AW-1:0]     jtag_out_mem_raddr,  // índice de pixel (byte)
  output logic  [7:0]       jtag_out_mem_rdata   // dato 8 bits hacia JTAG
);

  // --------------------------------------------------------------------------
  // Multiplexor del core activo
  // --------------------------------------------------------------------------
  logic          out_we_core;
  logic [AW-1:0] out_addr_core;
  logic [31:0]   out_wdata_core;

  always_comb begin
    out_we_core    = 1'b0;
    out_addr_core  = '0;
    out_wdata_core = 32'd0;

    if (mode_simd_eff) begin
      out_we_core    = out_mem_we_simd;
      out_addr_core  = out_mem_addr_simd;
      out_wdata_core = out_mem_wdata_simd;
    end else begin
      out_we_core    = out_mem_we_seq;
      out_addr_core  = out_mem_addr_seq;
      out_wdata_core = out_mem_wdata_seq;
    end
  end

  // --------------------------------------------------------------------------
  // FIFO de escrituras hacia mem_out
  // --------------------------------------------------------------------------
  localparam int OUT_FIFO_AW = $clog2(OUT_FIFO_DEPTH);

  typedef struct packed {
    logic [AW-1:0] addr;
    logic [31:0]   data;
  } out_fifo_entry_t;

  out_fifo_entry_t out_fifo   [OUT_FIFO_DEPTH-1:0];
  logic [OUT_FIFO_AW-1:0] out_fifo_wr_ptr, out_fifo_rd_ptr;
  logic [OUT_FIFO_AW:0]   out_fifo_count;

  wire out_fifo_full  = (out_fifo_count == OUT_FIFO_DEPTH);
  wire out_fifo_empty = (out_fifo_count == 0);

  // Pulso de dequeue generado por el writer
  logic deq_pulse;

  // Escritura en FIFO desde el core + gestión de punteros/contador
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      out_fifo_wr_ptr <= '0;
      out_fifo_rd_ptr <= '0;
      out_fifo_count  <= '0;
    end else begin
      // Enqueue cuando out_we_core=1 y el FIFO no está lleno
      if (out_we_core && !out_fifo_full) begin
        out_fifo[out_fifo_wr_ptr].addr <= out_addr_core;
        out_fifo[out_fifo_wr_ptr].data <= out_wdata_core;
        out_fifo_wr_ptr <= out_fifo_wr_ptr + 1'b1;
        out_fifo_count  <= out_fifo_count  + 1'b1;
      end

      // Dequeue cuando el writer ha completado una escritura en mem_out
      if (deq_pulse && !out_fifo_empty) begin
        out_fifo_rd_ptr <= out_fifo_rd_ptr + 1'b1;
        out_fifo_count  <= out_fifo_count  - 1'b1;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Writer hacia mem_out: saca del FIFO y genera peticiones de escritura
  // --------------------------------------------------------------------------
  logic          w_req_valid;
  logic [AW-1:0] w_req_addr;
  logic [31:0]   w_req_wdata;
  logic          w_req_ready;

  logic          w_ongoing;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      w_ongoing <= 1'b0;
      deq_pulse <= 1'b0;
    end else begin
      // Por defecto, el pulso de dequeue dura un ciclo
      deq_pulse <= 1'b0;

      // Inicia una nueva escritura si no hay ninguna en curso y el FIFO no está vacío
      if (!w_ongoing && !out_fifo_empty) begin
        w_ongoing <= 1'b1;
      end

      // Cuando handshake con mem_out se completa, se pasa al siguiente elemento
      if (w_ongoing && w_req_valid && w_req_ready) begin
        w_ongoing <= 1'b0;
        deq_pulse <= 1'b1;   // pedir al bloque del FIFO que haga rd_ptr++ y count--
      end
    end
  end

  // Señales del writer hacia mem_out
  assign w_req_valid = w_ongoing;
  assign w_req_addr  = out_fifo[out_fifo_rd_ptr].addr;
  assign w_req_wdata = out_fifo[out_fifo_rd_ptr].data;

  // --------------------------------------------------------------------------
  // Reader desde mem_out para JTAG: vista a bytes (8 bits/pixel)
  // --------------------------------------------------------------------------
  logic [AW-1:0] jtag_out_addr_reg;
  logic  [1:0]   jtag_out_lane;
  logic  [7:0]   out_byte_reg;

  typedef enum logic [1:0] { R_IDLE, R_REQ, R_WAIT } r_state_t;
  r_state_t r_state;

  logic          r_req_valid;
  logic          r_req_ready;
  logic [AW-1:0] r_req_addr;
  logic          r_resp_valid;
  logic [31:0]   r_resp_rdata;

  // FSM de lectura
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      r_state           <= R_IDLE;
      jtag_out_addr_reg <= '0;
      jtag_out_lane     <= 2'd0;
      out_byte_reg      <= 8'd0;
    end else begin
      case (r_state)
        R_IDLE: begin
          // Detectar cambio de dirección pedido por JTAG
          if (jtag_out_mem_raddr != jtag_out_addr_reg) begin
            jtag_out_addr_reg <= jtag_out_mem_raddr;
            jtag_out_lane     <= jtag_out_mem_raddr[1:0];
            r_state           <= R_REQ;
          end
        end

        R_REQ: begin
          // Esperar handshake de petición de lectura
          if (r_req_valid && r_req_ready) begin
            r_state <= R_WAIT;
          end
        end

        R_WAIT: begin
          // Esperar dato desde mem_out
          if (r_resp_valid) begin
            case (jtag_out_lane)
              2'd0: out_byte_reg <= r_resp_rdata[7:0];
              2'd1: out_byte_reg <= r_resp_rdata[15:8];
              2'd2: out_byte_reg <= r_resp_rdata[23:16];
              default: out_byte_reg <= r_resp_rdata[31:24];
            endcase
            r_state <= R_IDLE;
          end
        end

        default: r_state <= R_IDLE;
      endcase
    end
  end

  // Señales combinacionales del reader
  assign r_req_addr   = jtag_out_addr_reg[AW-1:2];  // índice de palabra de 32 bits
  assign r_req_valid  = (r_state == R_REQ);
  assign r_resp_valid = mem_out_resp_valid;
  assign r_resp_rdata = mem_out_resp_rdata;

  // Byte hacia JTAG
  assign jtag_out_mem_rdata = out_byte_reg;

  // --------------------------------------------------------------------------
  // Árbitro entre writer y reader para mem_out
  // --------------------------------------------------------------------------
  always_comb begin
    mem_out_req_valid = 1'b0;
    mem_out_req_we    = 1'b0;
    mem_out_req_addr  = '0;
    mem_out_req_wdata = 32'd0;

    w_req_ready       = 1'b0;
    r_req_ready       = 1'b0;

    // Prioridad: primero escrituras del core (writer), luego lecturas de JTAG
    if (w_req_valid) begin
      mem_out_req_valid = 1'b1;
      mem_out_req_we    = 1'b1;
      mem_out_req_addr  = w_req_addr;
      mem_out_req_wdata = w_req_wdata;
      w_req_ready       = mem_out_req_ready;
    end else if (r_req_valid) begin
      mem_out_req_valid = 1'b1;
      mem_out_req_we    = 1'b0;
      mem_out_req_addr  = r_req_addr;
      mem_out_req_wdata = 32'd0;
      r_req_ready       = mem_out_req_ready;
    end
  end

endmodule
