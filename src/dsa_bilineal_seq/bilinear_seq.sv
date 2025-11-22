`timescale 1ns/1ps

// ============================================================================
// bilinear_seq.sv  — Núcleo secuencial con interpolación bilineal Q8.8 real.
//
// Implementa exactamente el modelo discreto definido en Q8.8/Q0.16:
//
// - Dimensiones de salida:
//     out_w = floor((i_in_w * i_scale_q88) / 256)
//     out_h = floor((i_in_h * i_scale_q88) / 256)
//
// - Coordenadas fuente (por acumulación):
//     inv_scale_q88 = floor(65536 / i_scale_q88)  // 1/s en Q8.8
//     sx_fix, sy_fix en Q16.8
//     sx_fix_{n+1} = sx_fix_n + inv_scale_q88
//     sy_fix_{m+1} = sy_fix_m + inv_scale_q88
//     x0 = sx_fix[23:8],  fx_q = sx_fix[7:0]
//     y0 = sy_fix[23:8],  fy_q = sy_fix[7:0]
//
// - Pesos bilineales:
//     fx_q, fy_q en Q0.8  → wx*, wy* en Q0.8  → w_ij en Q0.16
//
// - Píxel final:
//     sum = Σ (w_ij * p_ij)   // Q8.16
//     PIX = round(sum / 2^16) // +0x8000 y luego >>16
//
// Con el mismo modelo en software, el resultado es bit a bit idéntico.
// Además, este núcleo sirve como referencia para el SIMD: bilinear_simd4
// replica exactamente estas fórmulas, pero procesando 4 píxeles en paralelo.
// ============================================================================

module bilinear_seq #(
  parameter int AW = 19
)(
  input  logic        clk,
  input  logic        rst_n,

  // Control
  input  logic        start,
  output logic        busy,
  output logic        done,

  // Stepping (modo paso a paso opcional)
  input  logic        i_step_en,      // si=1, pausa tras cada píxel
  input  logic        i_step_pulse,   // avanzar un píxel cuando step_en=1

  // Dimensiones entrada y escala Q8.8
  input  logic [15:0] i_in_w,
  input  logic [15:0] i_in_h,
  input  logic [15:0] i_scale_q88,   // Q8.8, >0

  // Dimensiones salida reportadas
  output logic [15:0] o_out_w,
  output logic [15:0] o_out_h,

  // Lectura fuente (single-read port)
  output logic [AW-1:0] in_raddr,
  input  logic [7:0]    in_rdata,

  // Escritura destino
  output logic [AW-1:0] out_waddr,
  output logic [7:0]    out_wdata,
  output logic          out_we,

  // Performance counters (por ejecución)
  output logic [31:0]  o_flop_count,
  output logic [31:0]  o_mem_rd_count,
  output logic [31:0]  o_mem_wr_count
);

  // ----------------- Constantes -----------------
  // 1.0 en Q0.8
  localparam logic [8:0]  ONE_Q08          = 9'd256;

  // Conteo aproximado de FLOPs por píxel: 8 mul + 3 sumas
  localparam logic [31:0] FLOPS_PER_PIXEL  = 32'd11;

  // Redondeo al pasar de Q8.16 a entero de 8 bits
  localparam logic [31:0] ROUND_Q016       = 32'h0000_8000;

  // ----------------- Registros internos -----------------

  // Dimensiones de salida (enteras)
  logic [15:0] out_w_reg, out_h_reg;

  // Coordenadas de salida (ox_cur, oy_cur)
  logic [15:0] ox_cur, oy_cur;

  // sx_fix, sy_fix se tratan como Q16.8: [23:8] entero, [7:0] fracción
  logic [23:0] sx_fix, sy_fix;

  // inv_scale_q88 representa 1/scale en Q8.8
  logic [15:0] inv_scale_q88;

  // Partes fraccionarias y enteras de sx_fix/sy_fix
  logic [7:0]  ax_q, ay_q;
  logic [15:0] sx_int, sy_int;

  // Coordenadas fuente base y fracciones clampadas
  logic [15:0] xi_base, yi_base;
  logic [7:0]  fx_q, fy_q;

  // Versión "next" para calcular coordenadas clampadas antes de registrarlas
  logic [15:0] xi_base_next, yi_base_next;
  logic [7:0]  fx_q_next, fy_q_next;

  // Intensidades fuente
  logic [7:0] I00, I10, I01, I11;

  // Pesos Q0.8 → productos Q0.16 (9+9 bits)
  logic [8:0]  wx0_ext, wx1_ext, wy0_ext, wy1_ext;
  logic [17:0] w00_q016, w10_q016, w01_q016, w11_q016;

  // Etapa de multiplicación (pipeline stage 1)
  logic [31:0] p00_r, p10_r, p01_r, p11_r;

  // Etapa de suma + redondeo (pipeline stage 2)
  logic [31:0] sum_q016;
  logic [31:0] sum_rounded;
  logic [7:0]  PIX_next;

  // ----------------- FSM -----------------
  localparam logic [3:0] S_IDLE        = 4'd0;
  localparam logic [3:0] S_INIT        = 4'd1;
  localparam logic [3:0] S_ROW_INIT    = 4'd2;
  localparam logic [3:0] S_PIXEL_START = 4'd3;
  localparam logic [3:0] S_READ00      = 4'd4;
  localparam logic [3:0] S_READ10      = 4'd5;
  localparam logic [3:0] S_READ01      = 4'd6;
  localparam logic [3:0] S_READ11      = 4'd7;
  localparam logic [3:0] S_MUL         = 4'd8;
  localparam logic [3:0] S_WRITE       = 4'd9;
  localparam logic [3:0] S_STEP_WAIT   = 4'd10;
  localparam logic [3:0] S_ADVANCE     = 4'd11;
  localparam logic [3:0] S_DONE        = 4'd12;

  logic [3:0] state, state_n;

  // --------------------------------------------------------------------------
  // Funciones auxiliares
  // --------------------------------------------------------------------------

  // Conversión (x,y,width) → dirección lineal
  function automatic logic [AW-1:0] linaddr(
    input logic [15:0] x,
    input logic [15:0] y,
    input logic [15:0] width
  );
    logic [31:0] tmp;
  begin
    tmp     = (y * width) + x;
    linaddr = tmp[AW-1:0];
  end
  endfunction

  // inv_q88: calcula 1/scale_q88 en Q8.8 como floor(65536 / scale_q88)
  function automatic logic [15:0] inv_q88(input logic [15:0] scale_q88);
    logic [31:0] num;
  begin
    if (scale_q88 == 16'd0) begin
      inv_q88 = 16'hFFFF;
    end else begin
      num     = 32'd65536;           // 256 * 256
      inv_q88 = (num / scale_q88);   // floor(65536 / scale_q88)
    end
  end
  endfunction

  // Productos W*H en Q8.8 (combinacional) para out_w/out_h
  logic [31:0] mul_w, mul_h;
  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  // Derivados de sx_fix/sy_fix (Q16.8)
  assign ax_q   = sx_fix[7:0];
  assign ay_q   = sy_fix[7:0];
  assign sx_int = sx_fix[23:8];
  assign sy_int = sy_fix[23:8];

  // Pesos 1D y 2D en Q0.8/Q0.16
  assign wx0_ext = ONE_Q08 - {1'b0, fx_q};
  assign wx1_ext = {1'b0, fx_q};
  assign wy0_ext = ONE_Q08 - {1'b0, fy_q};
  assign wy1_ext = {1'b0, fy_q};

  assign w00_q016 = wx0_ext * wy0_ext;
  assign w10_q016 = wx1_ext * wy0_ext;
  assign w01_q016 = wx0_ext * wy1_ext;
  assign w11_q016 = wx1_ext * wy1_ext;

  // Suma bilineal y redondeo en Q8.16 → entero 8 bits
  assign sum_q016    = p00_r + p10_r + p01_r + p11_r;
  assign sum_rounded = sum_q016 + ROUND_Q016;
  assign PIX_next    = sum_rounded[23:16];

  // --------------------------------------------------------------------------
  // Secuencia principal (FSM + datapath)
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state         <= S_IDLE;
      busy          <= 1'b0;
      done          <= 1'b0;
      out_we        <= 1'b0;

      out_waddr     <= '0;
      out_wdata     <= 8'h00;
      in_raddr      <= '0;

      ox_cur        <= 16'd0;
      oy_cur        <= 16'd0;

      out_w_reg     <= 16'd0;
      out_h_reg     <= 16'd0;
      inv_scale_q88 <= 16'd0;

      sx_fix        <= 24'd0;
      sy_fix        <= 24'd0;

      o_out_w       <= 16'd0;
      o_out_h       <= 16'd0;

      xi_base       <= 16'd0;
      yi_base       <= 16'd0;
      fx_q          <= 8'd0;
      fy_q          <= 8'd0;

      xi_base_next  <= 16'd0;
      yi_base_next  <= 16'd0;
      fx_q_next     <= 8'd0;
      fy_q_next     <= 8'd0;

      I00 <= 8'd0; I10 <= 8'd0; I01 <= 8'd0; I11 <= 8'd0;

      p00_r <= 32'd0; p10_r <= 32'd0; p01_r <= 32'd0; p11_r <= 32'd0;

      o_flop_count   <= 32'd0;
      o_mem_rd_count <= 32'd0;
      o_mem_wr_count <= 32'd0;
    end else begin
      state  <= state_n;
      out_we <= 1'b0; // por defecto

      case (state)
        // ---------------------------------------------------
        // S_IDLE: espera de start, cálculo inicial de tamaños
        // ---------------------------------------------------
        S_IDLE: begin
          done <= 1'b0;
          busy <= 1'b0;
          if (start) begin
            // Dimensiones de salida
            out_w_reg     <= mul_w[23:8];
            out_h_reg     <= mul_h[23:8];
            o_out_w       <= mul_w[23:8];
            o_out_h       <= mul_h[23:8];

            // 1/s en Q8.8
            inv_scale_q88 <= inv_q88(i_scale_q88);

            // Reinicio contadores
            o_flop_count   <= 32'd0;
            o_mem_rd_count <= 32'd0;
            o_mem_wr_count <= 32'd0;
          end
        end

        // ---------------------------------------------------
        // S_INIT: inicio de recorrido, primera fila y pixel (0,0)
        // ---------------------------------------------------
        S_INIT: begin
          busy   <= 1'b1;
          ox_cur <= 16'd0;
          oy_cur <= 16'd0;
          sx_fix <= 24'd0;
          sy_fix <= 24'd0;
        end

        // ---------------------------------------------------
        // S_ROW_INIT: al pasar a una nueva fila Y de salida
        // ---------------------------------------------------
        S_ROW_INIT: begin
          ox_cur <= 16'd0;
          sx_fix <= 24'd0;
        end

        // ---------------------------------------------------
        // S_PIXEL_START:
        //   - Calcula coordenadas fuente (sx,sy) a partir de
        //     sx_fix/sy_fix, aplica clamping y dispara la
        //     primera lectura I00.
        // ---------------------------------------------------
        S_PIXEL_START: begin
          // Coordenadas base sin clamping
          xi_base_next = sx_int;
          yi_base_next = sy_int;
          fx_q_next    = ax_q;
          fy_q_next    = ay_q;

          // Clamping en X
          if (sx_int >= i_in_w - 16'd1) begin
            xi_base_next = i_in_w - 16'd2;
            fx_q_next    = 8'hFF;
          end
          // Clamping en Y
          if (sy_int >= i_in_h - 16'd1) begin
            yi_base_next = i_in_h - 16'd2;
            fy_q_next    = 8'hFF;
          end

          // Registro de las coordenadas clampadas
          xi_base <= xi_base_next;
          yi_base <= yi_base_next;
          fx_q    <= fx_q_next;
          fy_q    <= fy_q_next;

          // Lectura I00 con coordenadas clampadas
          in_raddr <= linaddr(xi_base_next, yi_base_next, i_in_w);
        end

        // ---------------------------------------------------
        // S_READ00..S_READ11:
        //   - Realizan las 4 lecturas I00/I10/I01/I11 desde la
        //     BRAM en 4 ciclos consecutivos.
        // ---------------------------------------------------
        S_READ00: begin
          I00            <= in_rdata;
          in_raddr       <= linaddr(xi_base + 16'd1, yi_base, i_in_w);
          o_mem_rd_count <= o_mem_rd_count + 32'd1;
        end

        S_READ10: begin
          I10            <= in_rdata;
          in_raddr       <= linaddr(xi_base, yi_base + 16'd1, i_in_w);
          o_mem_rd_count <= o_mem_rd_count + 32'd1;
        end

        S_READ01: begin
          I01            <= in_rdata;
          in_raddr       <= linaddr(xi_base + 16'd1, yi_base + 16'd1, i_in_w);
          o_mem_rd_count <= o_mem_rd_count + 32'd1;
        end

        S_READ11: begin
          I11            <= in_rdata;
          o_mem_rd_count <= o_mem_rd_count + 32'd1;
        end

        // ---------------------------------------------------
        // S_MUL:
        //   - Productos w_ij * I_ij en Q8.16.
        // ---------------------------------------------------
        S_MUL: begin
          p00_r <= w00_q016 * I00;
          p10_r <= w10_q016 * I10;
          p01_r <= w01_q016 * I01;
          p11_r <= w11_q016 * I11;
        end

        // ---------------------------------------------------
        // S_WRITE:
        //   - Suma bilineal, redondeo y escritura de un píxel
        //     de salida.
        //   - Actualiza contadores de FLOPs y escrituras.
        // ---------------------------------------------------
        S_WRITE: begin
          out_waddr      <= linaddr(ox_cur, oy_cur, out_w_reg);
          out_wdata      <= PIX_next;
          out_we         <= 1'b1;

          o_mem_wr_count <= o_mem_wr_count + 32'd1;
          o_flop_count   <= o_flop_count + FLOPS_PER_PIXEL;
        end

        // ---------------------------------------------------
        // S_STEP_WAIT:
        //   - Modo debug: espera a que llegue i_step_pulse
        //     para avanzar al siguiente píxel.
        // ---------------------------------------------------
        S_STEP_WAIT: begin
          // No se modifica nada; la transición se controla en state_n
        end

        // ---------------------------------------------------
        // S_ADVANCE:
        //   - Avanza al siguiente píxel en X o a la siguiente
        //     fila en Y, actualizando sx_fix y sy_fix.
        // ---------------------------------------------------
        S_ADVANCE: begin
          if (ox_cur + 16'd1 < out_w_reg) begin
            // Siguiente píxel en la misma fila
            ox_cur <= ox_cur + 16'd1;
            // sx_fix_{n+1} = sx_fix_n + inv_scale_q88
            sx_fix <= sx_fix + {8'd0, inv_scale_q88};
          end else begin
            // Fin de fila → siguiente fila si aún hay más
            if (oy_cur + 16'd1 < out_h_reg) begin
              oy_cur <= oy_cur + 16'd1;
              sx_fix <= 24'd0;
              // sy_fix_{m+1} = sy_fix_m + inv_scale_q88
              sy_fix <= sy_fix + {8'd0, inv_scale_q88};
            end
          end
        end

        // ---------------------------------------------------
        // S_DONE: señaliza fin de procesamiento
        // ---------------------------------------------------
        S_DONE: begin
          busy <= 1'b0;
          done <= 1'b1;
        end
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // Próximo estado (incluye stepping opcional)
  // --------------------------------------------------------------------------
  always_comb begin
    state_n = state;
    unique case (state)
      S_IDLE:         state_n = (start) ? S_INIT : S_IDLE;
      S_INIT:         state_n = S_ROW_INIT;
      S_ROW_INIT:     state_n = S_PIXEL_START;
      S_PIXEL_START:  state_n = S_READ00;
      S_READ00:       state_n = S_READ10;
      S_READ10:       state_n = S_READ01;
      S_READ01:       state_n = S_READ11;
      S_READ11:       state_n = S_MUL;
      S_MUL:          state_n = S_WRITE;

      S_WRITE: begin
        if (i_step_en) state_n = S_STEP_WAIT;
        else begin
          if ((ox_cur + 16'd1 >= out_w_reg) && (oy_cur + 16'd1 >= out_h_reg))
            state_n = S_DONE;
          else
            state_n = S_ADVANCE;
        end
      end

      S_STEP_WAIT: begin
        if (i_step_pulse) begin
          if ((ox_cur + 16'd1 >= out_w_reg) && (oy_cur + 16'd1 >= out_h_reg))
            state_n = S_DONE;
          else
            state_n = S_ADVANCE;
        end else state_n = S_STEP_WAIT;
      end

      S_ADVANCE: begin
        if ((ox_cur + 16'd1 >= out_w_reg) && (oy_cur + 16'd1 >= out_h_reg))
          state_n = S_DONE;
        else if (ox_cur + 16'd1 >= out_w_reg)
          state_n = S_ROW_INIT;
        else
          state_n = S_PIXEL_START;
      end

      S_DONE:         state_n = S_IDLE;
      default:        state_n = S_IDLE;
    endcase
  end

endmodule
