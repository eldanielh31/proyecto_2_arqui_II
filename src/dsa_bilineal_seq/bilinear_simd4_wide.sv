`timescale 1ps/1ps

// ============================================================================
// bilinear_simd4_wide.sv
//   - SIMD x4 bilinear interpolation con memoria ancha (32 bits = 4 píxeles)
//   - Lee directamente de una memoria wide de 32 bits (mem_in)
//   - Escribe directamente en una memoria wide de 32 bits (mem_out)
//     * Cada grupo SIMD (4 lanes) produce una palabra de 32 bits
//     * Cada palabra representa hasta 4 píxeles de una fila de salida
//   - Esta versión alinea la generación de coordenadas (sx, sy) con
//     bilinear_seq_wide para obtener resultados bit-idénticos.
// ============================================================================

module bilinear_simd4_wide #(
  parameter int AW     = 16,   // ancho de dirección para memoria wide
  parameter int IMG_W  = 512,  // ancho máximo de imagen (no usado en lógica)
  parameter int IMG_H  = 512   // alto máximo de imagen (no usado en lógica)
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

  // Interfaz de escritura de salida (memoria ancha, 32 bits = 4 píxeles)
  output logic [AW-1:0]     out_mem_addr,
  output logic [31:0]       out_mem_wdata,
  output logic              out_mem_we,

  // Contadores de desempeño
  output logic [31:0]  o_flop_count,
  output logic [31:0]  o_mem_rd_count, // cuenta 4 vecinos por píxel
  output logic [31:0]  o_mem_wr_count  // cuenta words (32 bits) escritos
);

  // --------------------------------------------------------------------------
  // Parámetros internos
  // --------------------------------------------------------------------------
  localparam logic [31:0] FLOPS_PER_PIXEL = 32'd11;
  localparam logic [8:0]  ONE_Q08         = 9'd256;
  localparam logic [31:0] ROUND_Q016      = 32'h0000_8000;

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
  // Config latcheada en start + dimensiones de salida
  // --------------------------------------------------------------------------
  logic [15:0] in_w_reg, in_h_reg;
  logic [15:0] out_w_reg, out_h_reg;
  logic [15:0] inv_scale_q88;
  logic [15:0] groups_per_row;         // palabras de salida por fila (ceil(out_w/4))
  logic [AW-1:0] words_per_row_reg;    // palabras por fila de ENTRADA

  logic [31:0] mul_w, mul_h;

  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  function automatic logic [15:0] inv_q88(input logic [15:0] scale_q88);
    logic [31:0] num;
  begin
    if (scale_q88 == 16'd0)
      inv_q88 = 16'hFFFF;
    else begin
      num     = 32'd65536; // 1.0 en Q16.16
      inv_q88 = (num / scale_q88);
    end
  end
  endfunction

  // --------------------------------------------------------------------------
  // Coordenadas de salida (oy, grupos en X) y coordenadas fuente
  //   - Se usa la MISMA relación que en bilinear_seq_wide:
  //       sx_fix32 = ox * inv_scale_q88;
  //       sy_fix32 = oy * inv_scale_q88;
  // --------------------------------------------------------------------------
  logic [15:0] oy_cur, group_x;

  // Para Y (común a los 4 lanes)
  logic [31:0] sy_fix32;
  logic [15:0] sy_int_row;
  logic [7:0]  fy_q_row;

  // Para X por lane
  logic [15:0] ox_lane       [0:3];
  logic [31:0] sx_fix32_lane [0:3];

  // Coordenadas fuente base por lane
  logic [15:0] xi_base_lane  [0:3];
  logic [15:0] yi_base_row;        // Y base común a los 4 lanes
  logic [7:0]  fx_q_lane     [0:3];
  // fy_q_row ya es común

  // Píxeles vecinos por lane (TL, TR, BL, BR)
  logic [7:0] I00[0:3], I10[0:3], I01[0:3], I11[0:3];

  // Pesos en X/Y
  logic [8:0] wx0_lane[0:3], wx1_lane[0:3];
  logic [8:0] wy0_row, wy1_row;
  logic [17:0] w00_lane[0:3], w10_lane[0:3], w01_lane[0:3], w11_lane[0:3];
  logic [31:0] p00_r[0:3], p10_r[0:3], p01_r[0:3], p11_r[0:3];
  logic [31:0] sum_lane[0:3], sum_rounded_lane[0:3];
  logic [7:0]  pix_lane[0:3];

  // Pesos en Y (comunes a los 4 lanes)
  assign wy0_row = ONE_Q08 - {1'b0, fy_q_row};
  assign wy1_row = {1'b0, fy_q_row};

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
  logic [1:0]   current_lane;     // lane 0..3
  logic [2:0]   lane_phase;       // 0..3 (lecturas dentro del lane)
  logic [2:0]   lane_phase_last;  // 1 ó 3 según se cruce de palabra
  logic         lane_need_extra;  // 1 si pixel_offset == 3

  // Palabras temporales para el lane actual
  logic [31:0] row0_word_tmp;
  logic [31:0] row0_next_word_tmp;
  logic [31:0] row1_word_tmp;
  logic [31:0] row1_next_word_tmp;

  logic [1:0]    pixel_offset_cur;
  logic [AW-1:0] word_addr_row0_cur, word_addr_row1_cur;
  logic [31:0]   word_addr_row0_full, word_addr_row1_full;

  assign pixel_offset_cur = xi_base_lane[current_lane][1:0];

  always_comb begin
    word_addr_row0_full = (yi_base_row * words_per_row_reg) +
                          (xi_base_lane[current_lane] >> 2);
    word_addr_row1_full = ((yi_base_row + 16'd1) * words_per_row_reg) +
                          (xi_base_lane[current_lane] >> 2);

    word_addr_row0_cur  = word_addr_row0_full[AW-1:0];
    word_addr_row1_cur  = word_addr_row1_full[AW-1:0];
  end

  // --------------------------------------------------------------------------
  // Variables auxiliares
  // --------------------------------------------------------------------------
  integer i;
  integer n_wr, n_pix;

  logic [15:0] yi_tmp;
  logic [15:0] xi_tmp_lane [0:3];
  logic [7:0]  fy_tmp;
  logic [7:0]  fx_tmp_lane [0:3];

  // --------------------------------------------------------------------------
  // Lógica secuencial principal (FSM)
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
      groups_per_row  <= 16'd0;
      words_per_row_reg <= '0;

      oy_cur          <= 16'd0;
      group_x         <= 16'd0;

      sy_fix32        <= 32'd0;
      sy_int_row      <= 16'd0;
      fy_q_row        <= 8'd0;

      yi_base_row     <= 16'd0;

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
        sx_fix32_lane[i]<= 32'd0;
      end

      out_mem_addr  <= '0;
      out_mem_wdata <= 32'd0;
      out_mem_we    <= 1'b0;

      o_flop_count   <= 32'd0;
      o_mem_rd_count <= 32'd0;
      o_mem_wr_count <= 32'd0;

    end else begin
      state <= next_state;

      // Por defecto
      in_req_valid <= 1'b0;
      out_mem_we   <= 1'b0;

      case (state)

        // -------------------------------------------------------------------
        // Espera de start
        // -------------------------------------------------------------------
        S_IDLE: begin
          done <= 1'b0;
          busy <= 1'b0;

          if (start) begin
            in_w_reg        <= i_in_w;
            in_h_reg        <= i_in_h;
            inv_scale_q88   <= inv_q88(i_scale_q88);

            out_w_reg       <= mul_w[23:8];
            out_h_reg       <= mul_h[23:8];
            o_out_w         <= mul_w[23:8];
            o_out_h         <= mul_h[23:8];

            groups_per_row    <= (mul_w[23:8] + 16'd3) >> 2; // palabras/row salida
            words_per_row_reg <= (i_in_w       + 16'd3) >> 2; // palabras/row entrada

            o_flop_count    <= 32'd0;
            o_mem_rd_count  <= 32'd0;
            o_mem_wr_count  <= 32'd0;
          end
        end

        // -------------------------------------------------------------------
        // Inicialización global
        // -------------------------------------------------------------------
        S_INIT: begin
          busy   <= 1'b1;
          oy_cur <= 16'd0;
          group_x <= 16'd0;
        end

        // -------------------------------------------------------------------
        // Inicio de nueva fila
        // -------------------------------------------------------------------
        S_ROW_INIT: begin
          group_x <= 16'd0;
        end

        // -------------------------------------------------------------------
        // Preparar grupo de 4 píxeles (lanes) en la fila actual
        //   - Coordenadas fuente calculadas igual que en bilinear_seq_wide:
//       sy_fix32 = oy_cur * inv_scale_q88;
//       sx_fix32_lane[i] = ox_lane[i] * inv_scale_q88;
// ---------------------------------------------------------------------------
        S_GROUP_SETUP: begin
          // Y común a los 4 lanes
          sy_fix32   = oy_cur * inv_scale_q88;
          sy_int_row = sy_fix32[23:8];
          fy_tmp     = sy_fix32[7:0];

          yi_tmp = sy_int_row;
          if (sy_int_row >= in_h_reg - 16'd1) begin
            yi_tmp = (in_h_reg > 16'd1) ? (in_h_reg - 16'd2) : 16'd0;
            fy_tmp = 8'hFF;
          end

          yi_base_row <= yi_tmp;
          fy_q_row    <= fy_tmp;

          // X y fracción por lane
          for (i = 0; i < 4; i = i + 1) begin
            // Coordenada X de salida del lane i
            ox_lane[i] <= (group_x << 2) + i[15:0];

            // Coordenada fuente en X (Q16.8), igual que en módulo secuencial
            sx_fix32_lane[i] = ((group_x << 2) + i[15:0]) * inv_scale_q88;
            xi_tmp_lane[i]   = sx_fix32_lane[i][23:8];
            fx_tmp_lane[i]   = sx_fix32_lane[i][7:0];

            // Clamp a borde derecho
            if (xi_tmp_lane[i] >= in_w_reg - 16'd1) begin
              xi_tmp_lane[i] = (in_w_reg > 16'd1) ? (in_w_reg - 16'd2) : 16'd0;
              fx_tmp_lane[i] = 8'hFF;
            end

            xi_base_lane[i] <= xi_tmp_lane[i];
            fx_q_lane[i]    <= fx_tmp_lane[i];
          end

          current_lane      <= 2'd0;
          lane_phase        <= 3'd0;
          lane_need_extra   <= 1'b0;
          lane_phase_last   <= 3'd0;

          row0_word_tmp      <= 32'd0;
          row0_next_word_tmp <= 32'd0;
          row1_word_tmp      <= 32'd0;
          row1_next_word_tmp <= 32'd0;
        end

        // -------------------------------------------------------------------
        // Petición de lectura para el lane actual
        // -------------------------------------------------------------------
        S_LANE_REQ: begin
          if (lane_phase == 3'd0) begin
            lane_need_extra <= (xi_base_lane[current_lane][1:0] == 2'b11);
            lane_phase_last <= (xi_base_lane[current_lane][1:0] == 2'b11) ? 3'd3
                                                                           : 3'd1;
          end

          if (in_req_ready) begin
            in_req_valid <= 1'b1;

            unique case (lane_phase)
              3'd0: in_req_addr <= word_addr_row0_cur;                         // fila 0, primera
              3'd1: begin
                if (lane_need_extra)
                  in_req_addr <= word_addr_row0_cur + {{(AW-1){1'b0}}, 1'b1}; // fila 0, segunda
                else
                  in_req_addr <= word_addr_row1_cur;                           // fila 1, primera
              end
              3'd2: in_req_addr <= word_addr_row1_cur;                         // fila 1, primera
              3'd3: in_req_addr <= word_addr_row1_cur + {{(AW-1){1'b0}}, 1'b1}; // fila 1, segunda
              default: in_req_addr <= word_addr_row0_cur;
            endcase
          end
        end

        // -------------------------------------------------------------------
        // Espera de dato de memoria para el lane actual
        // -------------------------------------------------------------------
        S_LANE_WAIT: begin
          if (in_resp_valid) begin
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

            if (lane_phase == lane_phase_last)
              lane_phase <= 3'd0;
            else
              lane_phase <= lane_phase + 3'd1;
          end
        end

        // -------------------------------------------------------------------
        // Construcción de vecinos I00..I11 para el lane actual
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

          // 4 vecinos por píxel
          o_mem_rd_count <= o_mem_rd_count + 32'd4;

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
        // Aritmética por los 4 lanes
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
        // Escritura de los 4 píxeles interpolados en una sola palabra de 32 bits
        // -------------------------------------------------------------------
        S_WRITE: begin
          n_wr  = 0;
          n_pix = 0;

          // palabra = oy * groups_per_row + group_x
          out_mem_addr <= (oy_cur * groups_per_row) + group_x;

          // Empaquetado 4 lanes → 4 bytes
          out_mem_wdata = 32'd0;

          if (ox_lane[0] < out_w_reg) begin
            out_mem_wdata[7:0]   <= pix_lane[0];
            n_pix = n_pix + 1;
          end
          if (ox_lane[1] < out_w_reg) begin
            out_mem_wdata[15:8]  <= pix_lane[1];
            n_pix = n_pix + 1;
          end
          if (ox_lane[2] < out_w_reg) begin
            out_mem_wdata[23:16] <= pix_lane[2];
            n_pix = n_pix + 1;
          end
          if (ox_lane[3] < out_w_reg) begin
            out_mem_wdata[31:24] <= pix_lane[3];
            n_pix = n_pix + 1;
          end

          if (n_pix > 0) begin
            out_mem_we <= 1'b1;
            n_wr       = 1;
          end

          // Contadores
          o_mem_wr_count <= o_mem_wr_count + n_wr;
          o_flop_count   <= o_flop_count   + (n_pix * FLOPS_PER_PIXEL);
        end

        // -------------------------------------------------------------------
        // Avance a siguiente grupo / fila
        // -------------------------------------------------------------------
        S_ADVANCE: begin
          if (group_x + 16'd1 < groups_per_row) begin
            group_x <= group_x + 16'd1;
          end else begin
            if (oy_cur + 16'd1 < out_h_reg) begin
              oy_cur <= oy_cur + 16'd1;
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

        S_STEP_WAIT: begin
          // Sin lógica extra
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
