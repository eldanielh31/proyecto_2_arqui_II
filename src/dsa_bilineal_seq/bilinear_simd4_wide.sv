`timescale 1ps/1ps

// ============================================================================
// bilinear_simd4_wide.sv
//   - SIMD x4 bilinear interpolation con memoria ancha (32 bits = 4 píxeles)
//   - Procesa 4 píxeles por grupo usando un único puerto de memoria (handshake)
//   - Semántica Q8.8 idéntica al modelo secuencial / referencia C++
//   - Sin mem_read_controller externo
// ============================================================================

module bilinear_simd4_wide #(
  parameter int AW     = 18,   // ancho de dirección para memoria wide
  parameter int IMG_W  = 512,  // ancho máximo de imagen de entrada (píxeles)
  parameter int IMG_H  = 512   // alto máximo de imagen de entrada (píxeles)
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

  // Parámetros de imagen
  input  logic [15:0] i_in_w,
  input  logic [15:0] i_in_h,
  input  logic [15:0] i_scale_q88,

  // Dimensiones de salida
  output logic [15:0] o_out_w,
  output logic [15:0] o_out_h,

  // Interfaz memoria de entrada (ancha, 1 puerto, solo lectura)
  output logic              in_req_valid,
  input  logic              in_req_ready,
  output logic [AW-1:0]     in_req_addr,
  input  logic              in_resp_valid,
  input  logic [31:0]       in_resp_rdata,

  // Interfaz de escritura de salida (4 píxeles SIMD)
  output logic [AW-1:0] out_waddr0,
  output logic [7:0]    out_wdata0,
  output logic          out_we0,

  output logic [AW-1:0] out_waddr1,
  output logic [7:0]    out_wdata1,
  output logic          out_we1,

  output logic [AW-1:0] out_waddr2,
  output logic [7:0]    out_wdata2,
  output logic          out_we2,

  output logic [AW-1:0] out_waddr3,
  output logic [7:0]    out_wdata3,
  output logic          out_we3,

  // Contadores de desempeño
  output logic [31:0]  o_flop_count,
  output logic [31:0]  o_mem_rd_count, // cuenta 4 vecinos por píxel
  output logic [31:0]  o_mem_wr_count
);

  // --------------------------------------------------------------------------
  // Parámetros internos
  // --------------------------------------------------------------------------
  localparam logic [31:0] FLOPS_PER_PIXEL = 32'd11;
  localparam logic [8:0]  ONE_Q08         = 9'd256;
  localparam logic [31:0] ROUND_Q016      = 32'h0000_8000;

  // Palabras (32 bits) por fila de la imagen de entrada
  localparam int WORDS_PER_ROW = (IMG_W + 3) >> 2;

  // --------------------------------------------------------------------------
  // FSM
  // --------------------------------------------------------------------------
  typedef enum logic [4:0] {
    S_IDLE,
    S_INIT,
    S_ROW_INIT,
    S_GROUP_SETUP,
    S_LANE_REQ,
    S_LANE_WAIT,
    S_LANE_BUILD,
    S_ARITH_ALL,
    S_WRITE,
    S_STEP_WAIT,
    S_ADVANCE,
    S_DONE
  } state_t;

  state_t state, next_state;

  // --------------------------------------------------------------------------
  // Dimensiones de salida y escala inversa
  // --------------------------------------------------------------------------
  logic [15:0] out_w_reg, out_h_reg;
  logic [31:0] mul_w, mul_h;

  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  logic [15:0] inv_scale_q88;
  logic [15:0] groups_per_row;

  // inv_scale_q88 = floor(65536 / scale_q88) = 1/scale en Q8.8
  function automatic logic [15:0] inv_q88(input logic [15:0] scale_q88);
    logic [31:0] num;
  begin
    if (scale_q88 == 16'd0)
      inv_q88 = 16'hFFFF;
    else begin
      num      = 32'd65536; // 1.0 en Q16.16
      inv_q88  = (num / scale_q88);
    end
  end
  endfunction

  // --------------------------------------------------------------------------
  // Coordenadas de salida (oy, grupos en X) y coordenadas Q16.8
  // --------------------------------------------------------------------------
  logic [15:0] oy_cur, group_x;

  // Q16.8 para Y de fila y X base de grupo
  logic [23:0] sy_fix_row, sx_fix_group;
  logic [23:0] inv_step_q168, four_step_q168;

  assign inv_step_q168  = {8'd0, inv_scale_q88};        // inv_scale_q88 en Q16.8
  assign four_step_q168 = inv_step_q168 << 2;           // 4 * inv_scale

  // --------------------------------------------------------------------------
  // Datos por lane
  // --------------------------------------------------------------------------
  logic [15:0] ox_lane[0:3];     // coordenada X de salida por lane
  logic [15:0] xi_base_lane[0:3];// coordenada X base en imagen fuente
  logic [15:0] yi_base_row;      // coordenada Y base común a los 4 lanes
  logic [7:0]  fx_q_lane[0:3];   // fracción en X por lane (Q0.8)
  logic [7:0]  fy_q_row;         // fracción en Y común (Q0.8)

  // Píxeles vecinos por lane (TL, TR, BL, BR)
  logic [7:0] I00[0:3], I10[0:3], I01[0:3], I11[0:3];

  // Aritmética por lane (pesos, productos, resultados)
  logic [8:0]  wx0_lane[0:3], wx1_lane[0:3];
  logic [8:0]  wy0_row, wy1_row;
  logic [17:0] w00_lane[0:3], w10_lane[0:3], w01_lane[0:3], w11_lane[0:3];
  logic [31:0] p00_r[0:3], p10_r[0:3], p01_r[0:3], p11_r[0:3];
  logic [31:0] sum_lane[0:3], sum_rounded_lane[0:3];
  logic [7:0]  pix_lane[0:3];

  // Coordenadas fuente en Y (fila actual)
  logic [15:0] sy_int_row;
  logic [7:0]  ay_q_row;

  assign sy_int_row = sy_fix_row[23:8];
  assign ay_q_row   = sy_fix_row[7:0];

  // Pesos en Y (comunes a los 4 lanes)
  assign wy0_row = ONE_Q08 - {1'b0, fy_q_row};
  assign wy1_row = {1'b0, fy_q_row};

  // Pesos y sumas por lane
  genvar gv;
  generate
    for (gv = 0; gv < 4; gv = gv + 1) begin : g_lane_calc
      assign wx0_lane[gv] = ONE_Q08 - {1'b0, fx_q_lane[gv]};
      assign wx1_lane[gv] = {1'b0, fx_q_lane[gv]};

      assign w00_lane[gv] = wx0_lane[gv] * wy0_row;
      assign w10_lane[gv] = wx1_lane[gv] * wy0_row;
      assign w01_lane[gv] = wx0_lane[gv] * wy1_row;
      assign w11_lane[gv] = wx1_lane[gv] * wy1_row;

      assign sum_lane[gv]         = p00_r[gv] + p10_r[gv] + p01_r[gv] + p11_r[gv];
      assign sum_rounded_lane[gv] = sum_lane[gv] + ROUND_Q016;
      assign pix_lane[gv]         = sum_rounded_lane[gv][23:16];
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Control de lanes y acceso a memoria wide (1 puerto)
  // --------------------------------------------------------------------------
  logic [1:0] current_lane;        // lane 0..3
  logic [2:0] lane_phase;          // 0..3 (lecturas dentro del lane)
  logic [2:0] lane_phase_last;     // 1 ó 3 según se cruce de palabra
  logic       lane_need_extra;     // 1 si pixel_offset == 3

  // Palabras temporales para el lane actual
  logic [31:0] row0_word_tmp;
  logic [31:0] row0_next_word_tmp;
  logic [31:0] row1_word_tmp;
  logic [31:0] row1_next_word_tmp;

  logic [1:0]  pixel_offset_cur;
  logic [AW-1:0] word_addr_row0_cur, word_addr_row1_cur;

  assign pixel_offset_cur = xi_base_lane[current_lane][1:0];

  // Cálculo de dirección de palabra (32 bits) por fila para el lane actual
  always_comb begin
    word_addr_row0_cur = (yi_base_row * WORDS_PER_ROW) +
                         (xi_base_lane[current_lane] >> 2);
    word_addr_row1_cur = ((yi_base_row + 16'd1) * WORDS_PER_ROW) +
                         (xi_base_lane[current_lane] >> 2);
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
    tmp     = (y * width) + x;
    linaddr = tmp[AW-1:0];
  end
  endfunction

  // --------------------------------------------------------------------------
  // Variables auxiliares
  // --------------------------------------------------------------------------
  integer i;
  integer n_wr, n_pix;

  logic [23:0] sx_loc;
  logic [15:0] yi_tmp, xi_tmp_lane[0:3];
  logic [7:0]  fy_tmp, fx_tmp_lane[0:3];

  // --------------------------------------------------------------------------
  // Lógica secuencial principal (FSM)
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // Reset síncrono
      state         <= S_IDLE;
      busy          <= 1'b0;
      done          <= 1'b0;

      out_w_reg     <= 16'd0;
      out_h_reg     <= 16'd0;
      o_out_w       <= 16'd0;
      o_out_h       <= 16'd0;

      inv_scale_q88  <= 16'd0;
      groups_per_row <= 16'd0;

      oy_cur        <= 16'd0;
      group_x       <= 16'd0;
      sy_fix_row    <= 24'd0;
      sx_fix_group  <= 24'd0;

      yi_base_row   <= 16'd0;
      fy_q_row      <= 8'd0;

      current_lane      <= 2'd0;
      lane_phase        <= 3'd0;
      lane_phase_last   <= 3'd0;
      lane_need_extra   <= 1'b0;

      row0_word_tmp      <= 32'd0;
      row0_next_word_tmp <= 32'd0;
      row1_word_tmp      <= 32'd0;
      row1_next_word_tmp <= 32'd0;

      in_req_valid <= 1'b0;
      in_req_addr  <= '0;

      for (i = 0; i < 4; i = i + 1) begin
        ox_lane[i]      <= 16'd0;
        xi_base_lane[i] <= 16'd0;
        fx_q_lane[i]    <= 8'd0;
        I00[i]          <= 8'd0;
        I10[i]          <= 8'd0;
        I01[i]          <= 8'd0;
        I11[i]          <= 8'd0;
        p00_r[i]        <= 32'd0;
        p10_r[i]        <= 32'd0;
        p01_r[i]        <= 32'd0;
        p11_r[i]        <= 32'd0;
      end

      out_we0 <= 1'b0; out_we1 <= 1'b0;
      out_we2 <= 1'b0; out_we3 <= 1'b0;

      o_flop_count   <= 32'd0;
      o_mem_rd_count <= 32'd0;
      o_mem_wr_count <= 32'd0;

    end else begin
      state <= next_state;

      // Valores por defecto cada ciclo
      out_we0      <= 1'b0;
      out_we1      <= 1'b0;
      out_we2      <= 1'b0;
      out_we3      <= 1'b0;
      in_req_valid <= 1'b0;

      case (state)

        // -------------------------------------------------------------------
        // Espera de start: calcula dimensiones y escala inversa
        // -------------------------------------------------------------------
        S_IDLE: begin
          done <= 1'b0;
          busy <= 1'b0;

          if (start) begin
            out_w_reg      <= mul_w[23:8];
            out_h_reg      <= mul_h[23:8];
            o_out_w        <= mul_w[23:8];
            o_out_h        <= mul_h[23:8];
            inv_scale_q88  <= inv_q88(i_scale_q88);
            groups_per_row <= (mul_w[23:8] + 16'd3) >> 2;

            o_flop_count   <= 32'd0;
            o_mem_rd_count <= 32'd0;
            o_mem_wr_count <= 32'd0;
          end
        end

        // -------------------------------------------------------------------
        // Inicialización global
        // -------------------------------------------------------------------
        S_INIT: begin
          busy         <= 1'b1;
          oy_cur       <= 16'd0;
          group_x      <= 16'd0;
          sy_fix_row   <= 24'd0;
          sx_fix_group <= 24'd0;
        end

        // -------------------------------------------------------------------
        // Inicio de nueva fila de salida
        // -------------------------------------------------------------------
        S_ROW_INIT: begin
          group_x      <= 16'd0;
          sx_fix_group <= 24'd0;
        end

        // -------------------------------------------------------------------
        // Preparar grupo de 4 píxeles (lanes) en la fila actual
        // -------------------------------------------------------------------
        S_GROUP_SETUP: begin
          // Clamp en Y (fila)
          yi_tmp = sy_int_row;
          fy_tmp = ay_q_row;
          if (sy_int_row >= i_in_h - 16'd1) begin
            yi_tmp = i_in_h - 16'd2;
            fy_tmp = 8'hFF;
          end
          yi_base_row <= yi_tmp;
          fy_q_row    <= fy_tmp;

          // X por lane (4 píxeles del grupo, usando sy_fix_row/sx_fix_group en Q16.8)
          for (i = 0; i < 4; i = i + 1) begin
            // Coordenada X de salida
            ox_lane[i] <= (group_x << 2) + i[15:0];

            // Coordenada fuente en X = sx_fix_group + i * inv_step_q168
            unique case (i)
              0: sx_loc = sx_fix_group;
              1: sx_loc = sx_fix_group + inv_step_q168;
              2: sx_loc = sx_fix_group + (inv_step_q168 << 1);
              3: sx_loc = sx_fix_group + inv_step_q168 + (inv_step_q168 << 1);
              default: sx_loc = sx_fix_group;
            endcase

            xi_tmp_lane[i] = sx_loc[23:8];
            fx_tmp_lane[i] = sx_loc[7:0];

            // Clamp en X
            if (xi_tmp_lane[i] >= i_in_w - 16'd1) begin
              xi_tmp_lane[i] = i_in_w - 16'd2;
              fx_tmp_lane[i] = 8'hFF;
            end

            xi_base_lane[i] <= xi_tmp_lane[i];
            fx_q_lane[i]    <= fx_tmp_lane[i];
          end

          // Preparar primer lane
          current_lane      <= 2'd0;
          lane_phase        <= 3'd0;
          lane_need_extra   <= 1'b0;
          lane_phase_last   <= 3'd0;

          // Limpiar temporales de palabras
          row0_word_tmp      <= 32'd0;
          row0_next_word_tmp <= 32'd0;
          row1_word_tmp      <= 32'd0;
          row1_next_word_tmp <= 32'd0;
        end

        // -------------------------------------------------------------------
        // Petición de lectura para el lane actual (1 puerto wide)
        // -------------------------------------------------------------------
        S_LANE_REQ: begin
          // Si es el inicio del lane (phase=0), definir si se cruza de palabra
          if (lane_phase == 3'd0) begin
            lane_need_extra <= (xi_base_lane[current_lane][1:0] == 2'b11);
            lane_phase_last <= (xi_base_lane[current_lane][1:0] == 2'b11) ? 3'd3
                                                                           : 3'd1;
          end

          if (in_req_ready) begin
            in_req_valid <= 1'b1;

            unique case (lane_phase)
              3'd0: begin
                // Fila 0, primera palabra
                in_req_addr <= word_addr_row0_cur;
              end
              3'd1: begin
                if (lane_need_extra) begin
                  // Fila 0, segunda palabra
                  in_req_addr <= word_addr_row0_cur + {{(AW-1){1'b0}}, 1'b1};
                end else begin
                  // Fila 1, primera palabra (no hay extra en fila 0)
                  in_req_addr <= word_addr_row1_cur;
                end
              end
              3'd2: begin
                // Fila 1, primera palabra (cuando sí hay extra en fila 0)
                in_req_addr <= word_addr_row1_cur;
              end
              3'd3: begin
                // Fila 1, segunda palabra (cuando sí hay extra)
                in_req_addr <= word_addr_row1_cur + {{(AW-1){1'b0}}, 1'b1};
              end
              default: begin
                in_req_addr <= word_addr_row0_cur;
              end
            endcase
          end
        end

        // -------------------------------------------------------------------
        // Espera de dato de memoria para el lane actual
        // -------------------------------------------------------------------
        S_LANE_WAIT: begin
          if (in_resp_valid) begin
            // Guardar palabra según fase
            unique case (lane_phase)
              3'd0: row0_word_tmp      <= in_resp_rdata;
              3'd1: begin
                if (lane_need_extra)
                  row0_next_word_tmp <= in_resp_rdata;
                else
                  row1_word_tmp      <= in_resp_rdata;
              end
              3'd2: row1_word_tmp      <= in_resp_rdata;
              3'd3: row1_next_word_tmp <= in_resp_rdata;
              default: ;
            endcase

            // ¿Es la última lectura requerida para este lane?
            if (lane_phase == lane_phase_last) begin
              lane_phase <= 3'd0; // se reinicia para el siguiente uso
            end else begin
              lane_phase <= lane_phase + 3'd1;
            end
          end
        end

        // -------------------------------------------------------------------
        // Construcción de vecinos I00..I11 para el lane actual
        //   (se hace en un ciclo separado para evitar uso de datos "stale")
        // -------------------------------------------------------------------
        S_LANE_BUILD: begin
          unique case (pixel_offset_cur)
            2'b00: begin
              I00[current_lane] <= row0_word_tmp[7:0];
              I10[current_lane] <= row0_word_tmp[15:8];
              I01[current_lane] <= row1_word_tmp[7:0];
              I11[current_lane] <= row1_word_tmp[15:8];
            end
            2'b01: begin
              I00[current_lane] <= row0_word_tmp[15:8];
              I10[current_lane] <= row0_word_tmp[23:16];
              I01[current_lane] <= row1_word_tmp[15:8];
              I11[current_lane] <= row1_word_tmp[23:16];
            end
            2'b10: begin
              I00[current_lane] <= row0_word_tmp[23:16];
              I10[current_lane] <= row0_word_tmp[31:24];
              I01[current_lane] <= row1_word_tmp[23:16];
              I11[current_lane] <= row1_word_tmp[31:24];
            end
            2'b11: begin
              I00[current_lane] <= row0_word_tmp[31:24];
              I10[current_lane] <= row0_next_word_tmp[7:0];
              I01[current_lane] <= row1_word_tmp[31:24];
              I11[current_lane] <= row1_next_word_tmp[7:0];
            end
            default: begin
              I00[current_lane] <= 8'd0;
              I10[current_lane] <= 8'd0;
              I01[current_lane] <= 8'd0;
              I11[current_lane] <= 8'd0;
            end
          endcase

          // 4 vecinos por lane
          o_mem_rd_count <= o_mem_rd_count + 32'd4;

          // Preparar siguiente lane (si lo hay)
          if (current_lane != 2'd3) begin
            current_lane      <= current_lane + 2'd1;
            lane_phase        <= 3'd0;
            row0_word_tmp      <= 32'd0;
            row0_next_word_tmp <= 32'd0;
            row1_word_tmp      <= 32'd0;
            row1_next_word_tmp <= 32'd0;
          end
        end

        // -------------------------------------------------------------------
        // Aritmética por los 4 lanes (se hace en paralelo)
        // -------------------------------------------------------------------
        S_ARITH_ALL: begin
          for (i = 0; i < 4; i = i + 1) begin
            p00_r[i] <= w00_lane[i] * I00[i];
            p10_r[i] <= w10_lane[i] * I10[i];
            p01_r[i] <= w01_lane[i] * I01[i];
            p11_r[i] <= w11_lane[i] * I11[i];
          end
        end

        // -------------------------------------------------------------------
        // Escritura de los 4 píxeles interpolados
        // -------------------------------------------------------------------
        S_WRITE: begin
          n_wr  = 0;
          n_pix = 0;

          if (ox_lane[0] < out_w_reg) begin
            out_waddr0 <= linaddr(ox_lane[0], oy_cur, out_w_reg);
            out_wdata0 <= pix_lane[0];
            out_we0    <= 1'b1;
            n_wr       = n_wr  + 1;
            n_pix      = n_pix + 1;
          end

          if (ox_lane[1] < out_w_reg) begin
            out_waddr1 <= linaddr(ox_lane[1], oy_cur, out_w_reg);
            out_wdata1 <= pix_lane[1];
            out_we1    <= 1'b1;
            n_wr       = n_wr  + 1;
            n_pix      = n_pix + 1;
          end

          if (ox_lane[2] < out_w_reg) begin
            out_waddr2 <= linaddr(ox_lane[2], oy_cur, out_w_reg);
            out_wdata2 <= pix_lane[2];
            out_we2    <= 1'b1;
            n_wr       = n_wr  + 1;
            n_pix      = n_pix + 1;
          end

          if (ox_lane[3] < out_w_reg) begin
            out_waddr3 <= linaddr(ox_lane[3], oy_cur, out_w_reg);
            out_wdata3 <= pix_lane[3];
            out_we3    <= 1'b1;
            n_wr       = n_wr  + 1;
            n_pix      = n_pix + 1;
          end

          o_mem_wr_count <= o_mem_wr_count + n_wr;
          o_flop_count   <= o_flop_count   + (n_pix * FLOPS_PER_PIXEL);
        end

        // -------------------------------------------------------------------
        // Avance a siguiente grupo / fila
        // -------------------------------------------------------------------
        S_ADVANCE: begin
          if (group_x + 16'd1 < groups_per_row) begin
            group_x      <= group_x + 16'd1;
            sx_fix_group <= sx_fix_group + four_step_q168;
          end else begin
            if (oy_cur + 16'd1 < out_h_reg) begin
              oy_cur     <= oy_cur + 16'd1;
              sy_fix_row <= sy_fix_row + inv_step_q168;
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

        // STEP WAIT: sin lógica secuencial adicional
        S_STEP_WAIT: begin
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
        next_state = S_GROUP_SETUP;
      end

      S_GROUP_SETUP: begin
        next_state = S_LANE_REQ;
      end

      S_LANE_REQ: begin
        if (in_req_ready)
          next_state = S_LANE_WAIT;
      end

      S_LANE_WAIT: begin
        if (in_resp_valid) begin
          if (lane_phase == lane_phase_last)
            next_state = S_LANE_BUILD;
          else
            next_state = S_LANE_REQ;
        end
      end

      S_LANE_BUILD: begin
        // Si ya se procesó el lane 3, se pasa a aritmética
        if (current_lane == 2'd3)
          next_state = S_ARITH_ALL;
        else
          next_state = S_LANE_REQ;
      end

      S_ARITH_ALL: begin
        next_state = S_WRITE;
      end

      S_WRITE: begin
        if (i_step_en)
          next_state = S_STEP_WAIT;
        else if ((group_x + 16'd1 >= groups_per_row) &&
                 (oy_cur   + 16'd1 >= out_h_reg))
          next_state = S_DONE;
        else
          next_state = S_ADVANCE;
      end

      S_STEP_WAIT: begin
        if (i_step_pulse) begin
          if ((group_x + 16'd1 >= groups_per_row) &&
              (oy_cur   + 16'd1 >= out_h_reg))
            next_state = S_DONE;
          else
            next_state = S_ADVANCE;
        end
      end

      S_ADVANCE: begin
        if ((group_x + 16'd1 >= groups_per_row) &&
            (oy_cur   + 16'd1 >= out_h_reg))
          next_state = S_DONE;
        else if (group_x + 16'd1 >= groups_per_row)
          next_state = S_ROW_INIT;
        else
          next_state = S_GROUP_SETUP;
      end

      S_DONE: begin
        next_state = S_IDLE;
      end

      default: next_state = S_IDLE;
    endcase
  end

endmodule
