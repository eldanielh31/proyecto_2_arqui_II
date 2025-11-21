`timescale 1ns/1ps

// ============================================================================
// bilinear_simd4.sv  — Núcleo SIMD x4 para interpolación bilineal Q8.8 real.
//
// - Misma matemática que bilinear_seq:
//     * out_w = floor((i_in_w * i_scale_q88) / 256)
//     * out_h = floor((i_in_h * i_scale_q88) / 256)
//     * inv_scale_q88 = floor(65536 / i_scale_q88)  // Q8.8
//     * Coordenadas fuente: sx,sy en Q16.8, clamp en bordes
//     * Pesos fx_q,fy_q -> w_ij en Q0.16
//     * sum = Σ(w_ij * p_ij) en Q8.16, redondeo +0x8000 >>16
//
// - Procesa grupos de 4 píxeles (lanes 0..3) por vez:
//     * lane_x[i] = group_ox + i
//     * lane_valid[i] = (lane_x[i] < out_w_reg)
// - Todos los lanes usan la MISMA lógica de coordenadas que el secuencial.
//
// - La BRAM de entrada es 1R síncrona (latencia 1 ciclo):
//     * Cada patrón de acceso usa estados: *_INIT -> *_WAIT -> *_DATA
//       para evitar capturar datos desfasados.
//
// Este diseño es más verboso pero mucho más robusto y elimina el problema
// de un lane con valor constante (columnas "dc").
// ============================================================================

module bilinear_simd4 #(
  parameter int AW = 19
)(
  input  logic        clk,
  input  logic        rst_n,

  // Control
  input  logic        start,
  output logic        busy,
  output logic        done,

  // Stepping (no usado en HW final)
  input  logic        i_step_en,
  input  logic        i_step_pulse,

  // Dimensiones de entrada y escala Q8.8
  input  logic [15:0] i_in_w,
  input  logic [15:0] i_in_h,
  input  logic [15:0] i_scale_q88,

  // Dimensiones de salida (reportadas al wrapper)
  output logic [15:0] o_out_w,
  output logic [15:0] o_out_h,

  // Memoria de entrada (1 puerto de lectura)
  output logic [AW-1:0] in_raddr,
  input  logic [7:0]    in_rdata,

  // Memoria de salida (1 puerto de escritura)
  output logic [AW-1:0] out_waddr,
  output logic [7:0]    out_wdata,
  output logic          out_we,

  // Contadores de performance
  output logic [31:0]  o_flop_count,
  output logic [31:0]  o_mem_rd_count,
  output logic [31:0]  o_mem_wr_count
);

  localparam int       LANES            = 4;
  localparam logic[8:0]  ONE_Q08        = 9'd256;
  localparam logic[31:0] FLOPS_PER_PIXEL= 32'd11;
  localparam logic[31:0] ROUND_Q016     = 32'h0000_8000;

  // --------------------------------------------------------------------------
  // Dimensiones de salida e inversa de escala
  // --------------------------------------------------------------------------
  logic [15:0] out_w_reg, out_h_reg;
  logic [15:0] inv_scale_q88;

  // Productos W,H * scale_q88 (Q8.8)
  logic [31:0] mul_w, mul_h;
  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  // Inv_q88 idéntica a bilinear_seq
  function automatic logic [15:0] inv_q88(input logic [15:0] scale_q88);
    logic [31:0] num;
  begin
    if (scale_q88 == 16'd0) begin
      inv_q88 = 16'hFFFF;
    end else begin
      num     = 32'd65536;          // 256 * 256
      inv_q88 = (num / scale_q88);  // floor(65536 / scale_q88)
    end
  end
  endfunction

  // --------------------------------------------------------------------------
  // Función de direccionamiento lineal (igual a bilinear_seq)
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
  // Coordenadas de salida
  // --------------------------------------------------------------------------
  logic [15:0] oy_cur;      // fila de salida actual
  logic [15:0] group_ox;    // x base del grupo (0,4,8,...)

  // Coordenadas x por lane
  logic [15:0] lane_x     [LANES];
  logic        lane_valid [LANES];

  // Coordenadas fuente X por lane
  logic [15:0] xi_base    [LANES];  // entero fuente X
  logic [7:0]  fx_q       [LANES];  // fracción X (Q0.8)

  // Coordenadas fuente Y (comunes a los 4 lanes)
  logic [15:0] yi_base_row;
  logic [7:0]  fy_q_row;

  // Cálculo de sy (para la fila actual)
  logic [31:0] sy_mul;
  logic [23:0] sy_fix_row;
  logic [15:0] sy_int_row;
  logic [7:0]  ay_row;

  // Cálculo de sx por lane
  logic [31:0] sx_mul_lane;
  logic [23:0] sx_fix_lane;
  logic [15:0] sx_int_lane;
  logic [7:0]  ax_lane;

  // --------------------------------------------------------------------------
  // Intensidades por lane
  // --------------------------------------------------------------------------
  logic [7:0] I00[LANES];
  logic [7:0] I10[LANES];
  logic [7:0] I01[LANES];
  logic [7:0] I11[LANES];

  // Pesos por lane
  logic [8:0] wx0_ext[LANES];
  logic [8:0] wx1_ext[LANES];
  logic [8:0] wy0_ext;
  logic [8:0] wy1_ext;

  logic [17:0] w00_q016[LANES];
  logic [17:0] w10_q016[LANES];
  logic [17:0] w01_q016[LANES];
  logic [17:0] w11_q016[LANES];

  // Productos por lane (Q8.16)
  logic [31:0] p00_r[LANES];
  logic [31:0] p10_r[LANES];
  logic [31:0] p01_r[LANES];
  logic [31:0] p11_r[LANES];

  // Suma y redondeo por lane
  logic [31:0] sum_q016_lane   [LANES];
  logic [31:0] sum_rounded_lane[LANES];
  logic [7:0]  PIX_lane        [LANES];

  // Pesos en Y (comunes)
  assign wy0_ext = ONE_Q08 - {1'b0, fy_q_row};
  assign wy1_ext = {1'b0, fy_q_row};

  // Generación de pesos y suma por lane
  genvar g;
  generate
    for (g = 0; g < LANES; g++) begin : GEN_WEIGHTS
      assign wx0_ext[g] = ONE_Q08 - {1'b0, fx_q[g]};
      assign wx1_ext[g] = {1'b0, fx_q[g]};

      assign w00_q016[g] = wx0_ext[g] * wy0_ext;
      assign w10_q016[g] = wx1_ext[g] * wy0_ext;
      assign w01_q016[g] = wx0_ext[g] * wy1_ext;
      assign w11_q016[g] = wx1_ext[g] * wy1_ext;

      assign sum_q016_lane[g]    = p00_r[g] + p10_r[g] + p01_r[g] + p11_r[g];
      assign sum_rounded_lane[g] = sum_q016_lane[g] + ROUND_Q016;
      assign PIX_lane[g]         = sum_rounded_lane[g][23:16];
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Contadores de performance
  // --------------------------------------------------------------------------
  logic [31:0] flops_incr;
  integer k_fl;

  always_comb begin
    flops_incr = 32'd0;
    for (k_fl = 0; k_fl < LANES; k_fl = k_fl + 1) begin
      if (lane_valid[k_fl])
        flops_incr = flops_incr + FLOPS_PER_PIXEL;
    end
  end

  // --------------------------------------------------------------------------
  // FSM
  // --------------------------------------------------------------------------
  typedef enum logic [5:0] {
    S_IDLE        = 6'd0,
    S_INIT        = 6'd1,
    S_ROW_INIT    = 6'd2,
    S_GROUP_SETUP = 6'd3,

    // Lectura de I00 (xi, yi)
    S_G00_INIT    = 6'd4,
    S_G00_WAIT    = 6'd5,
    S_G00_DATA    = 6'd6,

    // Lectura de I10 (xi+1, yi)
    S_G10_INIT    = 6'd7,
    S_G10_WAIT    = 6'd8,
    S_G10_DATA    = 6'd9,

    // Lectura de I01 (xi, yi+1)
    S_G01_INIT    = 6'd10,
    S_G01_WAIT    = 6'd11,
    S_G01_DATA    = 6'd12,

    // Lectura de I11 (xi+1, yi+1)
    S_G11_INIT    = 6'd13,
    S_G11_WAIT    = 6'd14,
    S_G11_DATA    = 6'd15,

    // Cálculo y escritura
    S_MUL         = 6'd16,
    S_WRITE0      = 6'd17,
    S_WRITE1      = 6'd18,
    S_WRITE2      = 6'd19,
    S_WRITE3      = 6'd20,
    S_NEXT_GROUP  = 6'd21,
    S_NEXT_ROW    = 6'd22,
    S_DONE        = 6'd23
  } state_t;

  state_t state, state_n;

  // Índice de lane para lecturas secuenciales
  logic [1:0] lane_idx;

  // --------------------------------------------------------------------------
  // Próximo estado
  // --------------------------------------------------------------------------
  always_comb begin
    state_n = state;

    unique case (state)
      S_IDLE: begin
        if (start) state_n = S_INIT;
      end

      S_INIT:        state_n = S_ROW_INIT;
      S_ROW_INIT:    state_n = S_GROUP_SETUP;

      S_GROUP_SETUP: state_n = S_G00_INIT;

      // I00
      S_G00_INIT:    state_n = S_G00_WAIT;
      S_G00_WAIT:    state_n = S_G00_DATA;
      S_G00_DATA: begin
        if (lane_idx == 2'd3)
          state_n = S_G10_INIT;
        else
          state_n = S_G00_DATA;
      end

      // I10
      S_G10_INIT:    state_n = S_G10_WAIT;
      S_G10_WAIT:    state_n = S_G10_DATA;
      S_G10_DATA: begin
        if (lane_idx == 2'd3)
          state_n = S_G01_INIT;
        else
          state_n = S_G10_DATA;
      end

      // I01
      S_G01_INIT:    state_n = S_G01_WAIT;
      S_G01_WAIT:    state_n = S_G01_DATA;
      S_G01_DATA: begin
        if (lane_idx == 2'd3)
          state_n = S_G11_INIT;
        else
          state_n = S_G01_DATA;
      end

      // I11
      S_G11_INIT:    state_n = S_G11_WAIT;
      S_G11_WAIT:    state_n = S_G11_DATA;
      S_G11_DATA: begin
        if (lane_idx == 2'd3)
          state_n = S_MUL;
        else
          state_n = S_G11_DATA;
      end

      S_MUL:    state_n = S_WRITE0;
      S_WRITE0: state_n = S_WRITE1;
      S_WRITE1: state_n = S_WRITE2;
      S_WRITE2: state_n = S_WRITE3;
      S_WRITE3: state_n = S_NEXT_GROUP;

      S_NEXT_GROUP: begin
        if ((group_ox + LANES[15:0]) >= out_w_reg)
          state_n = S_NEXT_ROW;
        else
          state_n = S_GROUP_SETUP;
      end

      S_NEXT_ROW: begin
        if ((oy_cur + 16'd1) >= out_h_reg)
          state_n = S_DONE;
        else
          state_n = S_ROW_INIT;
      end

      S_DONE: begin
        state_n = S_IDLE;
      end

      default: state_n = S_IDLE;
    endcase
  end

  // --------------------------------------------------------------------------
  // Lógica secuencial
  // --------------------------------------------------------------------------
  integer i;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state        <= S_IDLE;
      busy         <= 1'b0;
      done         <= 1'b0;
      out_we       <= 1'b0;

      in_raddr     <= '0;
      out_waddr    <= '0;
      out_wdata    <= 8'h00;

      out_w_reg    <= 16'd0;
      out_h_reg    <= 16'd0;
      o_out_w      <= 16'd0;
      o_out_h      <= 16'd0;
      inv_scale_q88<= 16'd0;

      oy_cur       <= 16'd0;
      group_ox     <= 16'd0;

      yi_base_row  <= 16'd0;
      fy_q_row     <= 8'd0;

      for (i = 0; i < LANES; i = i + 1) begin
        lane_x[i]     <= 16'd0;
        lane_valid[i] <= 1'b0;
        xi_base[i]    <= 16'd0;
        fx_q[i]       <= 8'd0;
        I00[i]        <= 8'd0;
        I10[i]        <= 8'd0;
        I01[i]        <= 8'd0;
        I11[i]        <= 8'd0;
        p00_r[i]      <= 32'd0;
        p10_r[i]      <= 32'd0;
        p01_r[i]      <= 32'd0;
        p11_r[i]      <= 32'd0;
      end

      lane_idx        <= 2'd0;
      o_flop_count    <= 32'd0;
      o_mem_rd_count  <= 32'd0;
      o_mem_wr_count  <= 32'd0;

    end else begin
      state  <= state_n;
      out_we <= 1'b0;  // por defecto

      unique case (state)

        // -------------------------------------------------------
        S_IDLE: begin
          busy <= 1'b0;
          done <= 1'b0;
          if (start) begin
            // Calcular out_w/out_h e inv_scale_q88 (igual que bilinear_seq)
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

        // -------------------------------------------------------
        S_INIT: begin
          busy    <= 1'b1;
          oy_cur  <= 16'd0;
          group_ox<= 16'd0;
        end

        // -------------------------------------------------------
        // Inicialización de fila: calcular Y fuente (Q16.8)
        // -------------------------------------------------------
        S_ROW_INIT: begin
          group_ox <= 16'd0;

          sy_mul      = oy_cur * inv_scale_q88;
          sy_fix_row  = sy_mul[23:0];
          sy_int_row  = sy_fix_row[23:8];
          ay_row      = sy_fix_row[7:0];

          if (sy_int_row >= i_in_h - 16'd1) begin
            yi_base_row <= i_in_h - 16'd2;
            fy_q_row    <= 8'hFF;
          end else begin
            yi_base_row <= sy_int_row;
            fy_q_row    <= ay_row;
          end
        end

        // -------------------------------------------------------
        // Configuración de grupo: coordenadas X por lane
        // -------------------------------------------------------
        S_GROUP_SETUP: begin
          for (i = 0; i < LANES; i = i + 1) begin
            lane_x[i]     <= group_ox + i[15:0];
            lane_valid[i] <= ((group_ox + i[15:0]) < out_w_reg);

            // sx = lane_x * inv_scale_q88 (Q16.8)
            sx_mul_lane = (group_ox + i[15:0]) * inv_scale_q88;
            sx_fix_lane = sx_mul_lane[23:0];
            sx_int_lane = sx_fix_lane[23:8];
            ax_lane     = sx_fix_lane[7:0];

            if (sx_int_lane >= i_in_w - 16'd1) begin
              xi_base[i] <= i_in_w - 16'd2;
              fx_q[i]    <= 8'hFF;
            end else begin
              xi_base[i] <= sx_int_lane;
              fx_q[i]    <= ax_lane;
            end
          end
        end

        // -------------------------------------------------------
        // Lectura de I00 (x,y): 4 lanes con latencia 1 ciclo
        // -------------------------------------------------------
        S_G00_INIT: begin
          lane_idx <= 2'd0;
          in_raddr <= linaddr(xi_base[0], yi_base_row, i_in_w);
        end

        S_G00_WAIT: begin
          // sólo se espera un ciclo para que la BRAM cargue el primer dato
        end

        S_G00_DATA: begin
          // Captura para lane_idx actual
          if (lane_valid[lane_idx]) begin
            I00[lane_idx]    <= in_rdata;
            o_mem_rd_count   <= o_mem_rd_count + 32'd1;
          end

          if (lane_idx < 2'd3) begin
            lane_idx <= lane_idx + 2'd1;
            in_raddr <= linaddr(xi_base[lane_idx + 1], yi_base_row, i_in_w);
          end
        end

        // -------------------------------------------------------
        // Lectura de I10 (x+1,y)
        // -------------------------------------------------------
        S_G10_INIT: begin
          lane_idx <= 2'd0;
          in_raddr <= linaddr(xi_base[0] + 16'd1, yi_base_row, i_in_w);
        end

        S_G10_WAIT: begin
        end

        S_G10_DATA: begin
          if (lane_valid[lane_idx]) begin
            I10[lane_idx]    <= in_rdata;
            o_mem_rd_count   <= o_mem_rd_count + 32'd1;
          end

          if (lane_idx < 2'd3) begin
            lane_idx <= lane_idx + 2'd1;
            in_raddr <= linaddr(xi_base[lane_idx + 1] + 16'd1, yi_base_row, i_in_w);
          end
        end

        // -------------------------------------------------------
        // Lectura de I01 (x,y+1)
        // -------------------------------------------------------
        S_G01_INIT: begin
          lane_idx <= 2'd0;
          in_raddr <= linaddr(xi_base[0], yi_base_row + 16'd1, i_in_w);
        end

        S_G01_WAIT: begin
        end

        S_G01_DATA: begin
          if (lane_valid[lane_idx]) begin
            I01[lane_idx]    <= in_rdata;
            o_mem_rd_count   <= o_mem_rd_count + 32'd1;
          end

          if (lane_idx < 2'd3) begin
            lane_idx <= lane_idx + 2'd1;
            in_raddr <= linaddr(xi_base[lane_idx + 1], yi_base_row + 16'd1, i_in_w);
          end
        end

        // -------------------------------------------------------
        // Lectura de I11 (x+1,y+1)
        // -------------------------------------------------------
        S_G11_INIT: begin
          lane_idx <= 2'd0;
          in_raddr <= linaddr(xi_base[0] + 16'd1, yi_base_row + 16'd1, i_in_w);
        end

        S_G11_WAIT: begin
        end

        S_G11_DATA: begin
          if (lane_valid[lane_idx]) begin
            I11[lane_idx]    <= in_rdata;
            o_mem_rd_count   <= o_mem_rd_count + 32'd1;
          end

          if (lane_idx < 2'd3) begin
            lane_idx <= lane_idx + 2'd1;
            in_raddr <= linaddr(xi_base[lane_idx + 1] + 16'd1,
                                yi_base_row + 16'd1,
                                i_in_w);
          end
        end

        // -------------------------------------------------------
        // Multiplicaciones (todos los lanes en paralelo)
        // -------------------------------------------------------
        S_MUL: begin
          for (i = 0; i < LANES; i = i + 1) begin
            if (lane_valid[i]) begin
              p00_r[i] <= w00_q016[i] * I00[i];
              p10_r[i] <= w10_q016[i] * I10[i];
              p01_r[i] <= w01_q016[i] * I01[i];
              p11_r[i] <= w11_q016[i] * I11[i];
            end else begin
              p00_r[i] <= 32'd0;
              p10_r[i] <= 32'd0;
              p01_r[i] <= 32'd0;
              p11_r[i] <= 32'd0;
            end
          end
          o_flop_count <= o_flop_count + flops_incr;
        end

        // -------------------------------------------------------
        // Escritura de píxeles (un lane por ciclo, write-port único)
        // -------------------------------------------------------
        S_WRITE0: begin
          if (lane_valid[0]) begin
            out_waddr      <= linaddr(lane_x[0], oy_cur, out_w_reg);
            out_wdata      <= PIX_lane[0];
            out_we         <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 32'd1;
          end
        end

        S_WRITE1: begin
          if (lane_valid[1]) begin
            out_waddr      <= linaddr(lane_x[1], oy_cur, out_w_reg);
            out_wdata      <= PIX_lane[1];
            out_we         <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 32'd1;
          end
        end

        S_WRITE2: begin
          if (lane_valid[2]) begin
            out_waddr      <= linaddr(lane_x[2], oy_cur, out_w_reg);
            out_wdata      <= PIX_lane[2];
            out_we         <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 32'd1;
          end
        end

        S_WRITE3: begin
          if (lane_valid[3]) begin
            out_waddr      <= linaddr(lane_x[3], oy_cur, out_w_reg);
            out_wdata      <= PIX_lane[3];
            out_we         <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 32'd1;
          end
        end

        // -------------------------------------------------------
        // Avance de grupo y fila
        // -------------------------------------------------------
        S_NEXT_GROUP: begin
          group_ox <= group_ox + LANES[15:0];
        end

        S_NEXT_ROW: begin
          oy_cur <= oy_cur + 16'd1;
        end

        S_DONE: begin
          busy <= 1'b0;
          done <= 1'b1;
        end

        default: ;
      endcase
    end
  end

endmodule
