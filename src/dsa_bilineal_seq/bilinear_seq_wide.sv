`timescale 1ps/1ps

// ============================================================================
// bilinear_seq_wide.sv
//   - Bilinear secuencial con memoria ancha (32 bits = 4 píxeles)
//   - Lee directamente de una memoria wide de 32 bits (mem_in)
//   - Escribe directamente en una memoria wide de 32 bits (mem_out)
//     * Cada palabra de salida contiene 4 píxeles (4x8 bits)
//     * Se hace el empaquetado a 32 bits dentro del módulo
// ============================================================================

module bilinear_seq_wide #(
  parameter int AW     = 16,  // Ancho de dirección en PALABRAS (32 bits)
  parameter int IMG_W  = 512, // Ancho máximo de imagen de entrada (no usado en lógica)
  parameter int IMG_H  = 512  // Alto máximo de imagen de entrada (no usado en lógica)
)(
  input  logic        clk,
  input  logic        rst_n,

  // Control
  input  logic        start,
  output logic        busy,
  output logic        done,

  // Stepping (modo debug)
  input  logic        i_step_en,
  input  logic        i_step_pulse,

  // Parámetros de imagen (dinámicos, la única fuente de verdad)
  input  logic [15:0] i_in_w,        // ancho de entrada (píxeles)
  input  logic [15:0] i_in_h,        // alto de entrada  (píxeles)
  input  logic [15:0] i_scale_q88,   // escala Q8.8

  // Dimensiones de salida
  output logic [15:0] o_out_w,
  output logic [15:0] o_out_h,

  // Interfaz memoria de entrada (ancha, 1 puerto, solo lectura)
  output logic              in_req_valid,
  input  logic              in_req_ready,
  output logic [AW-1:0]     in_req_addr,
  input  logic              in_resp_valid,
  input  logic [31:0]       in_resp_rdata,

  // Interfaz de escritura de salida (memoria ancha, 32 bits = 4 píxeles)
  output logic [AW-1:0]     out_mem_addr,
  output logic [31:0]       out_mem_wdata,
  output logic              out_mem_we,

  // Contadores de desempeño
  output logic [31:0]       o_flop_count,
  output logic [31:0]       o_mem_rd_count,  // cuenta vecinos (4 por píxel)
  output logic [31:0]       o_mem_wr_count   // cuenta words (32 bits) escritos
);

  // --------------------------------------------------------------------------
  // Constantes internas
  // --------------------------------------------------------------------------
  localparam logic [31:0] FLOPS_PER_PIXEL = 32'd11;
  localparam logic [8:0]  ONE_Q08         = 9'd256;
  localparam logic [31:0] ROUND_Q016      = 32'h0000_8000;

  // --------------------------------------------------------------------------
  // FSM principal
  // --------------------------------------------------------------------------
  typedef enum logic [4:0] {
    S_IDLE,
    S_INIT,
    S_ROW_INIT,
    S_PIXEL_START,
    S_MEM_ROW0_0_REQ,
    S_MEM_ROW0_0_WAIT,
    S_MEM_ROW0_1_REQ,
    S_MEM_ROW0_1_WAIT,
    S_MEM_ROW1_0_REQ,
    S_MEM_ROW1_0_WAIT,
    S_MEM_ROW1_1_REQ,
    S_MEM_ROW1_1_WAIT,
    S_WRITE,
    S_STEP_WAIT,
    S_ADVANCE,
    S_DONE
  } state_t;

  state_t state, next_state;

  // --------------------------------------------------------------------------
  // Config latcheada en start + dimensiones de salida
  // --------------------------------------------------------------------------
  logic [15:0] in_w_reg, in_h_reg;
  logic [15:0] out_w_reg, out_h_reg;
  logic [15:0] inv_scale_q88;

  // Palabras por fila de ENTRADA (igual que antes)
  logic [AW-1:0] words_per_row_reg;

  // Palabras por fila de SALIDA (para mem_out en 32 bits)
  logic [AW-1:0] out_words_per_row_reg;

  logic [31:0] mul_w, mul_h;

  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  function automatic logic [15:0] inv_q88(input logic [15:0] scale_q88);
    logic [31:0] num;
  begin
    if (scale_q88 == 16'd0)
      inv_q88 = 16'hFFFF;
    else begin
      num     = 32'd65536; // 1.0 en Q16.16 -> 2^16
      inv_q88 = (num / scale_q88);
    end
  end
  endfunction

  // --------------------------------------------------------------------------
  // Coordenadas de salida (ox, oy) y fuente (sx, sy)
  // --------------------------------------------------------------------------
  logic [15:0] ox_cur, oy_cur;

  // Coordenadas fuente Q16.8
  logic [31:0] sx_fix32, sy_fix32;
  assign sx_fix32 = ox_cur * inv_scale_q88;
  assign sy_fix32 = oy_cur * inv_scale_q88;

  logic [15:0] sx_int, sy_int;
  logic [7:0]  ax_q, ay_q;

  assign sx_int = sx_fix32[23:8];
  assign sy_int = sy_fix32[23:8];
  assign ax_q   = sx_fix32[7:0];
  assign ay_q   = sy_fix32[7:0];

  // Coordenadas base clampadas y fracciones
  logic [15:0] xi_base, yi_base;
  logic [7:0]  fx_q, fy_q;

  logic [15:0] xi_base_next, yi_base_next;
  logic [7:0]  fx_q_next, fy_q_next;

  // --------------------------------------------------------------------------
  // Acceso a memoria ancha (32 bits) y vecinos TL/TR/BL/BR
  // --------------------------------------------------------------------------
  logic [31:0] row0_word, row0_next_word;
  logic [31:0] row1_word, row1_next_word;
  logic [1:0]  pixel_offset;
  logic        need_next_word;

  logic [AW-1:0] word_addr_row0, word_addr_row1;
  logic [31:0]   word_addr_row0_full, word_addr_row1_full;

  assign pixel_offset   = xi_base[1:0];
  assign need_next_word = (pixel_offset == 2'b11);

  always_comb begin
    word_addr_row0_full = (yi_base * words_per_row_reg) + (xi_base >> 2);
    word_addr_row1_full = ((yi_base + 16'd1) * words_per_row_reg) + (xi_base >> 2);

    word_addr_row0 = word_addr_row0_full[AW-1:0];
    word_addr_row1 = word_addr_row1_full[AW-1:0];
  end

  // --------------------------------------------------------------------------
  // Interpolación bilineal (en este módulo)
  // --------------------------------------------------------------------------
  logic [7:0] I00, I10, I01, I11;    // TL, TR, BL, BR
  logic [8:0] wx0, wx1, wy0, wy1;
  logic [17:0] w00, w10, w01, w11;
  logic [31:0] p00, p10, p01, p11;
  logic [31:0] sum_all, sum_rounded;
  logic [7:0]  interp_pixel;

  always_comb begin
    unique case (pixel_offset)
      2'b00: begin
        I00 = row0_word[7:0];
        I10 = row0_word[15:8];
        I01 = row1_word[7:0];
        I11 = row1_word[15:8];
      end
      2'b01: begin
        I00 = row0_word[15:8];
        I10 = row0_word[23:16];
        I01 = row1_word[15:8];
        I11 = row1_word[23:16];
      end
      2'b10: begin
        I00 = row0_word[23:16];
        I10 = row0_word[31:24];
        I01 = row1_word[23:16];
        I11 = row1_word[31:24];
      end
      2'b11: begin
        I00 = row0_word[31:24];
        I10 = row0_next_word[7:0];
        I01 = row1_word[31:24];
        I11 = row1_next_word[7:0];
      end
      default: begin
        I00 = 8'd0;
        I10 = 8'd0;
        I01 = 8'd0;
        I11 = 8'd0;
      end
    endcase
  end

  always_comb begin
    // Pesos en X e Y
    wy0 = ONE_Q08 - {1'b0, fy_q};
    wy1 = {1'b0, fy_q};
    wx0 = ONE_Q08 - {1'b0, fx_q};
    wx1 = {1'b0, fx_q};

    // Pesos combinados (Q16.16)
    w00 = wx0 * wy0;
    w10 = wx1 * wy0;
    w01 = wx0 * wy1;
    w11 = wx1 * wy1;

    // Productos con intensidades (Ixx en 8 bits)
    p00 = w00 * I00;
    p10 = w10 * I10;
    p01 = w01 * I01;
    p11 = w11 * I11;

    // Suma y redondeo
    sum_all      = p00 + p10 + p01 + p11;
    sum_rounded  = sum_all + ROUND_Q016;
    interp_pixel = sum_rounded[23:16];
  end

  // --------------------------------------------------------------------------
  // Empaquetado a 32 bits para mem_out
  //   - Se recorre la imagen en orden raster (ox, oy)
  //   - Se acumulan píxeles en registros hasta completar 4 por palabra
  //   - Se usa un mapeo lineal (row-major) por filas de salida:
//       * palabras por fila = ceil(out_w / 4)
//       * dirección de palabra = oy * out_words_per_row_reg + (ox >> 2)
// ---------------------------------------------------------------------------

  // Índice y dirección de palabra para el píxel actual
  logic [31:0] pix_index;
  logic [AW-1:0] cur_word_addr;
  logic [1:0]    cur_byte_off;

  // Registro de palabra pendiente (no escrita todavía)
  logic [31:0] out_word_data;
  logic [AW-1:0] out_word_addr;
  logic         out_word_valid;

  // Palabra "siguiente" (con el píxel actual actualizado)
  logic [31:0] next_word_data;

  // Cálculo de dirección de palabra de salida y byte dentro de la palabra
  always_comb begin
    // Índice lineal de píxel dentro de la imagen de salida
    pix_index    = (oy_cur * out_w_reg) + ox_cur;

    // Mapeo por filas con palabras de 4 píxeles:
    // palabra = oy * (palabras por fila) + floor(ox / 4)
    cur_word_addr = (oy_cur * out_words_per_row_reg) + (ox_cur >> 2);

    // Byte dentro de la palabra
    cur_byte_off  = ox_cur[1:0];
  end

  // Construcción de palabra con el píxel interpolado
  always_comb begin
    next_word_data = out_word_data;

    if (state == S_WRITE) begin
      unique case (cur_byte_off)
        2'b00: next_word_data[7:0]   = interp_pixel;
        2'b01: next_word_data[15:8]  = interp_pixel;
        2'b10: next_word_data[23:16] = interp_pixel;
        2'b11: next_word_data[31:24] = interp_pixel;
        default: ;
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // Lógica secuencial principal
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state           <= S_IDLE;
      busy            <= 1'b0;
      done            <= 1'b0;

      in_w_reg        <= 16'd0;
      in_h_reg        <= 16'd0;
      out_w_reg       <= 16'd0;
      out_h_reg       <= 16'd0;
      o_out_w         <= 16'd0;
      o_out_h         <= 16'd0;
      inv_scale_q88   <= 16'd0;
      words_per_row_reg     <= '0;
      out_words_per_row_reg <= '0;

      ox_cur          <= 16'd0;
      oy_cur          <= 16'd0;

      xi_base         <= 16'd0;
      yi_base         <= 16'd0;
      fx_q            <= 8'd0;
      fy_q            <= 8'd0;

      row0_word       <= 32'd0;
      row0_next_word  <= 32'd0;
      row1_word       <= 32'd0;
      row1_next_word  <= 32'd0;

      in_req_valid    <= 1'b0;
      in_req_addr     <= '0;

      // Memoria de salida (word pending y salida)
      out_word_data   <= 32'd0;
      out_word_addr   <= '0;
      out_word_valid  <= 1'b0;

      out_mem_addr    <= '0;
      out_mem_wdata   <= 32'd0;
      out_mem_we      <= 1'b0;

      o_flop_count    <= 32'd0;
      o_mem_rd_count  <= 32'd0;
      o_mem_wr_count  <= 32'd0;

    end else begin
      state        <= next_state;

      // Valores por defecto cada ciclo
      in_req_valid <= 1'b0;
      out_mem_we   <= 1'b0;

      case (state)
        // -------------------------------------------------------------------
        // Espera de start: calcula dimensiones y escala inversa
        // -------------------------------------------------------------------
        S_IDLE: begin
          done <= 1'b0;
          busy <= 1'b0;

          if (start) begin
            // Latch de configuración e inversa de escala
            in_w_reg      <= i_in_w;
            in_h_reg      <= i_in_h;
            inv_scale_q88 <= inv_q88(i_scale_q88);

            out_w_reg     <= mul_w[23:8];
            out_h_reg     <= mul_h[23:8];
            o_out_w       <= mul_w[23:8];
            o_out_h       <= mul_h[23:8];

            // Palabras por fila de entrada y salida (4 píxeles por word)
            words_per_row_reg     <= (i_in_w       + 16'd3) >> 2;
            out_words_per_row_reg <= (mul_w[23:8] + 16'd3) >> 2;

            // Contadores
            o_flop_count   <= 32'd0;
            o_mem_rd_count <= 32'd0;
            o_mem_wr_count <= 32'd0;

            // Estado de empaquetador de salida
            out_word_valid <= 1'b0;
            out_word_data  <= 32'd0;
            out_word_addr  <= '0;
          end
        end

        // -------------------------------------------------------------------
        // Inicialización global
        // -------------------------------------------------------------------
        S_INIT: begin
          busy  <= 1'b1;
          ox_cur <= 16'd0;
          oy_cur <= 16'd0;
        end

        // -------------------------------------------------------------------
        // Inicio de nueva fila de salida
        // -------------------------------------------------------------------
        S_ROW_INIT: begin
          ox_cur <= 16'd0;
        end

        // -------------------------------------------------------------------
        // Cálculo de coordenadas fuente y clamp
        // -------------------------------------------------------------------
        S_PIXEL_START: begin
          xi_base_next = sx_int;
          yi_base_next = sy_int;
          fx_q_next    = ax_q;
          fy_q_next    = ay_q;

          if (sx_int >= in_w_reg - 16'd1) begin
            xi_base_next = (in_w_reg > 16'd1) ? (in_w_reg - 16'd2) : 16'd0;
            fx_q_next    = 8'hFF;
          end

          if (sy_int >= in_h_reg - 16'd1) begin
            yi_base_next = (in_h_reg > 16'd1) ? (in_h_reg - 16'd2) : 16'd0;
            fy_q_next    = 8'hFF;
          end

          xi_base <= xi_base_next;
          yi_base <= yi_base_next;
          fx_q    <= fx_q_next;
          fy_q    <= fy_q_next;
        end

        // -------------------------------------------------------------------
        // Lecturas de fila 0
        // -------------------------------------------------------------------
        S_MEM_ROW0_0_REQ: begin
          if (in_req_ready) begin
            in_req_valid <= 1'b1;
            in_req_addr  <= word_addr_row0;
          end
        end

        S_MEM_ROW0_0_WAIT: begin
          if (in_resp_valid) begin
            row0_word <= in_resp_rdata;
          end
        end

        S_MEM_ROW0_1_REQ: begin
          if (in_req_ready) begin
            in_req_valid <= 1'b1;
            in_req_addr  <= word_addr_row0 + {{(AW-1){1'b0}}, 1'b1};
          end
        end

        S_MEM_ROW0_1_WAIT: begin
          if (in_resp_valid) begin
            row0_next_word <= in_resp_rdata;
          end
        end

        // -------------------------------------------------------------------
        // Lecturas de fila 1
        // -------------------------------------------------------------------
        S_MEM_ROW1_0_REQ: begin
          if (in_req_ready) begin
            in_req_valid <= 1'b1;
            in_req_addr  <= word_addr_row1;
          end
        end

        S_MEM_ROW1_0_WAIT: begin
          if (in_resp_valid) begin
            row1_word <= in_resp_rdata;
          end
        end

        S_MEM_ROW1_1_REQ: begin
          if (in_req_ready) begin
            in_req_valid <= 1'b1;
            in_req_addr  <= word_addr_row1 + {{(AW-1){1'b0}}, 1'b1};
          end
        end

        S_MEM_ROW1_1_WAIT: begin
          if (in_resp_valid) begin
            row1_next_word <= in_resp_rdata;
          end
        end

        // -------------------------------------------------------------------
        // Escritura de píxel interpolado → empaquetado en palabra de 32 bits
        // -------------------------------------------------------------------
        S_WRITE: begin
          // 1) Si hay una palabra pendiente con dirección distinta, primero se vacía
          if (out_word_valid && (out_word_addr != cur_word_addr)) begin
            out_mem_addr   <= out_word_addr;
            out_mem_wdata  <= out_word_data;
            out_mem_we     <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 32'd1;

            // Se inicia una nueva palabra para la dirección actual
            out_word_addr  <= cur_word_addr;
            out_word_data  <= 32'd0;
            out_word_valid <= 1'b1;
          end else if (!out_word_valid) begin
            // No hay palabra pendiente → se inicia para la palabra actual
            out_word_addr  <= cur_word_addr;
            out_word_data  <= 32'd0;
            out_word_valid <= 1'b1;
          end

          // 2) Se actualiza el contenido de la palabra con el píxel interpolado
          out_word_data <= next_word_data;

          // 3) Actualización de contadores por píxel
          o_mem_rd_count <= o_mem_rd_count + 32'd4;
          o_flop_count   <= o_flop_count   + FLOPS_PER_PIXEL;

          // 4) Si se completan los 4 bytes de la palabra (byte 3), se escribe
          if (cur_byte_off == 2'b11) begin
            out_mem_addr   <= cur_word_addr;
            out_mem_wdata  <= next_word_data;
            out_mem_we     <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 32'd1;
            out_word_valid <= 1'b0;
          end
        end

        // -------------------------------------------------------------------
        // Espera de paso manual (debug)
        // -------------------------------------------------------------------
        S_STEP_WAIT: begin
          // Sin lógica secuencial adicional; solo cambia estado
        end

        // -------------------------------------------------------------------
        // Avance a siguiente píxel / fila
        // -------------------------------------------------------------------
        S_ADVANCE: begin
          if (ox_cur + 16'd1 < out_w_reg) begin
            ox_cur <= ox_cur + 16'd1;
          end else begin
            if (oy_cur + 16'd1 < out_h_reg) begin
              oy_cur <= oy_cur + 16'd1;
              ox_cur <= 16'd0;
            end
          end
        end

        // -------------------------------------------------------------------
        // Fin de procesamiento
        // -------------------------------------------------------------------
        S_DONE: begin
          busy <= 1'b0;
          done <= 1'b1;

          // Si queda una palabra pendiente (parcial), se escribe una vez
          if (out_word_valid) begin
            out_mem_addr   <= out_word_addr;
            out_mem_wdata  <= out_word_data;
            out_mem_we     <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 32'd1;
            out_word_valid <= 1'b0;
          end
        end

        default: ;
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // Lógica de siguiente estado
  // --------------------------------------------------------------------------
  always_comb begin
    next_state = state;

    unique case (state)
      S_IDLE: begin
        if (start)
          next_state = S_INIT;
      end

      S_INIT: begin
        next_state = S_ROW_INIT;
      end

      S_ROW_INIT: begin
        next_state = S_PIXEL_START;
      end

      S_PIXEL_START: begin
        next_state = S_MEM_ROW0_0_REQ;
      end

      S_MEM_ROW0_0_REQ: begin
        if (in_req_ready)
          next_state = S_MEM_ROW0_0_WAIT;
      end

      S_MEM_ROW0_0_WAIT: begin
        if (in_resp_valid) begin
          if (need_next_word)
            next_state = S_MEM_ROW0_1_REQ;
          else
            next_state = S_MEM_ROW1_0_REQ;
        end
      end

      S_MEM_ROW0_1_REQ: begin
        if (in_req_ready)
          next_state = S_MEM_ROW0_1_WAIT;
      end

      S_MEM_ROW0_1_WAIT: begin
        if (in_resp_valid)
          next_state = S_MEM_ROW1_0_REQ;
      end

      S_MEM_ROW1_0_REQ: begin
        if (in_req_ready)
          next_state = S_MEM_ROW1_0_WAIT;
      end

      S_MEM_ROW1_0_WAIT: begin
        if (in_resp_valid) begin
          if (need_next_word)
            next_state = S_MEM_ROW1_1_REQ;
          else
            next_state = S_WRITE;
        end
      end

      S_MEM_ROW1_1_REQ: begin
        if (in_req_ready)
          next_state = S_MEM_ROW1_1_WAIT;
      end

      S_MEM_ROW1_1_WAIT: begin
        if (in_resp_valid)
          next_state = S_WRITE;
      end

      S_WRITE: begin
        if (i_step_en)
          next_state = S_STEP_WAIT;
        else if ((ox_cur + 16'd1 >= out_w_reg) &&
                 (oy_cur + 16'd1 >= out_h_reg))
          next_state = S_DONE;
        else
          next_state = S_ADVANCE;
      end

      S_STEP_WAIT: begin
        if (i_step_pulse) begin
          if ((ox_cur + 16'd1 >= out_w_reg) &&
              (oy_cur + 16'd1 >= out_h_reg))
            next_state = S_DONE;
          else
            next_state = S_ADVANCE;
        end
      end

      S_ADVANCE: begin
        if ((ox_cur + 16'd1 >= out_w_reg) &&
            (oy_cur + 16'd1 >= out_h_reg))
          next_state = S_DONE;
        else if (ox_cur + 16'd1 >= out_w_reg)
          next_state = S_ROW_INIT;
        else
          next_state = S_PIXEL_START;
      end

      S_DONE: begin
        next_state = S_IDLE;
      end

      default: next_state = S_IDLE;
    endcase
  end

endmodule
