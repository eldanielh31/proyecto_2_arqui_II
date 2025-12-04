`timescale 1ps/1ps

// ============================================================================
// bilinear_seq_wide.sv (versión simplificada)
//   - Bilinear secuencial con memoria ancha (32 bits = 4 píxeles)
//   - Acceso directo a memoria mediante interfaz valid/ready (1 puerto)
//   - Sin mem_read_controller ni bilinear_arith externos
// ============================================================================

module bilinear_seq_wide #(
  parameter int AW     = 18,  // Ancho de dirección para memoria ancha y salida
  parameter int IMG_W  = 512, // Ancho de imagen de entrada (en píxeles)
  parameter int IMG_H  = 512  // Alto de imagen de entrada (en píxeles)
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

  // Parámetros de imagen (dinámicos)
  input  logic [15:0] i_in_w,        // ancho de imagen de entrada (píxeles)
  input  logic [15:0] i_in_h,        // alto de imagen de entrada (píxeles)
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

  // Interfaz de escritura de salida (1 píxel por dirección)
  output logic [AW-1:0]     out_waddr,
  output logic [7:0]        out_wdata,
  output logic              out_we,

  // Contadores de desempeño
  output logic [31:0]       o_flop_count,
  output logic [31:0]       o_mem_rd_count,  // cuenta vecinos (4 por píxel)
  output logic [31:0]       o_mem_wr_count
);

  // --------------------------------------------------------------------------
  // Parámetros y constantes internas
  // --------------------------------------------------------------------------
  localparam logic [31:0] FLOPS_PER_PIXEL = 32'd11;
  localparam logic [8:0]  ONE_Q08         = 9'd256;
  localparam logic [31:0] ROUND_Q016      = 32'h0000_8000;

  // Palabras (32 bits) por fila de imagen de entrada
  localparam int WORDS_PER_ROW = (IMG_W + 3) >> 2;

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
  // Cálculo de dimensiones de salida
  // --------------------------------------------------------------------------
  logic [15:0] out_w_reg, out_h_reg;
  logic [31:0] mul_w, mul_h;

  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  // Escala inversa Q8.8
  logic [15:0] inv_scale_q88;

  function automatic logic [15:0] inv_q88(input logic [15:0] scale_q88);
    logic [31:0] num;
  begin
    if (scale_q88 == 16'd0)
      inv_q88 = 16'hFFFF;
    else begin
      num      = 32'd65536; // 1.0 en Q16.16 -> 2^16
      inv_q88  = (num / scale_q88);
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

  assign pixel_offset   = xi_base[1:0];
  assign need_next_word = (pixel_offset == 2'b11);

  // Cálculo de dirección de palabra (32 bits) por fila
  always_comb begin
    word_addr_row0 = (yi_base * WORDS_PER_ROW) + (xi_base >> 2);
    word_addr_row1 = ((yi_base + 16'd1) * WORDS_PER_ROW) + (xi_base >> 2);
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

  // Selección de vecinos según offset dentro de la palabra
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

  // Pesos Q8.8 y productos
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
    sum_all     = p00 + p10 + p01 + p11;
    sum_rounded = sum_all + ROUND_Q016;
    interp_pixel = sum_rounded[23:16];
  end

  // --------------------------------------------------------------------------
  // Función para dirección lineal de salida (1 píxel por dirección)
  // --------------------------------------------------------------------------
  function automatic logic [AW-1:0] linaddr(
    input logic [15:0] x,
    input logic [15:0] y,
    input logic [15:0] width
  );
    logic [31:0] tmp;
  begin
    tmp    = (y * width) + x;
    linaddr = tmp[AW-1:0];
  end
  endfunction

  // --------------------------------------------------------------------------
  // Lógica secuencial principal
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state         <= S_IDLE;
      busy          <= 1'b0;
      done          <= 1'b0;

      out_w_reg     <= 16'd0;
      out_h_reg     <= 16'd0;
      o_out_w       <= 16'd0;
      o_out_h       <= 16'd0;

      inv_scale_q88 <= 16'd0;

      ox_cur        <= 16'd0;
      oy_cur        <= 16'd0;

      xi_base       <= 16'd0;
      yi_base       <= 16'd0;
      fx_q          <= 8'd0;
      fy_q          <= 8'd0;

      row0_word     <= 32'd0;
      row0_next_word<= 32'd0;
      row1_word     <= 32'd0;
      row1_next_word<= 32'd0;

      out_waddr     <= '0;
      out_wdata     <= 8'd0;
      out_we        <= 1'b0;

      in_req_valid  <= 1'b0;
      in_req_addr   <= '0;

      o_flop_count  <= 32'd0;
      o_mem_rd_count<= 32'd0;
      o_mem_wr_count<= 32'd0;

    end else begin
      state        <= next_state;

      // Valores por defecto cada ciclo
      out_we       <= 1'b0;
      in_req_valid <= 1'b0;

      case (state)
        // -------------------------------------------------------------------
        // Espera de start: calcula dimensiones y escala inversa
        // -------------------------------------------------------------------
        S_IDLE: begin
          done <= 1'b0;
          busy <= 1'b0;

          if (start) begin
            out_w_reg     <= mul_w[23:8];
            out_h_reg     <= mul_h[23:8];
            o_out_w       <= mul_w[23:8];
            o_out_h       <= mul_h[23:8];
            inv_scale_q88 <= inv_q88(i_scale_q88);

            o_flop_count   <= 32'd0;
            o_mem_rd_count <= 32'd0;
            o_mem_wr_count <= 32'd0;
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
          // Coordenadas base sin clamp
          xi_base_next = sx_int;
          yi_base_next = sy_int;
          fx_q_next    = ax_q;
          fy_q_next    = ay_q;

          // Clamp en X
          if (sx_int >= i_in_w - 16'd1) begin
            xi_base_next = i_in_w - 16'd2;
            fx_q_next    = 8'hFF;
          end

          // Clamp en Y
          if (sy_int >= i_in_h - 16'd1) begin
            yi_base_next = i_in_h - 16'd2;
            fy_q_next    = 8'hFF;
          end

          xi_base <= xi_base_next;
          yi_base <= yi_base_next;
          fx_q    <= fx_q_next;
          fy_q    <= fy_q_next;
        end

        // -------------------------------------------------------------------
        // Lectura fila 0, primera palabra
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

        // -------------------------------------------------------------------
        // Lectura fila 0, segunda palabra (solo si offset == 3)
        // -------------------------------------------------------------------
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
        // Lectura fila 1, primera palabra
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

        // -------------------------------------------------------------------
        // Lectura fila 1, segunda palabra (solo si offset == 3)
        // -------------------------------------------------------------------
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
        // Escritura de píxel interpolado a memoria de salida
        // -------------------------------------------------------------------
        S_WRITE: begin
          out_waddr <= linaddr(ox_cur, oy_cur, out_w_reg);
          out_wdata <= interp_pixel;
          out_we    <= 1'b1;

          // Contadores (4 vecinos por píxel, FLOPs aproximados)
          o_mem_rd_count <= o_mem_rd_count + 32'd4;
          o_flop_count   <= o_flop_count   + FLOPS_PER_PIXEL;
          o_mem_wr_count <= o_mem_wr_count + 32'd1;
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
