`timescale 1ns/1ps

// ============================================================================
// bilinear_simd4.sv  — Núcleo SIMD x4 para interpolación bilineal Q8.8.
//
// Objetivo:
//   - Reproducir la misma interpolación Q8.8 de bilinear_seq,
//     pero procesando hasta 4 píxeles de salida por grupo (SIMD x4).
//
//   - Formato y fórmulas (idénticas al secuencial):
//       out_w = floor((i_in_w * i_scale_q88) / 256)
//       out_h = floor((i_in_h * i_scale_q88) / 256)
//       inv_scale_q88 = floor(65536 / i_scale_q88)
//
//       sx_fix, sy_fix en Q16.8 (24 bits): [23:8]=entero, [7:0]=fracción.
//
//       sum = Σ (w_ij * p_ij)   en Q8.16
//       PIX = round(sum / 2^16)  (+0x8000 y >>16)
//
// Interfaz de memoria:
//   - Lectura: se usan los puertos lane0_* como fuente efectiva (banco mem_in).
//     Los demás lanes se mantienen por compatibilidad de pines/BDF.
//   - Escritura: 4 puertos lógicos (out_waddr0..3 / out_wdata0..3 / out_we0..3).
//   - Lecturas en BRAM sincrónica de 1 ciclo:
//       * Estado *_A: se ponen direcciones.
//       * Estado *_D: se capturan datos.
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

  // Stepping (debug)
  input  logic        i_step_en,
  input  logic        i_step_pulse,

  // Dimensión de entrada y escala Q8.8
  input  logic [15:0] i_in_w,
  input  logic [15:0] i_in_h,
  input  logic [15:0] i_scale_q88,

  // Dimensión de salida
  output logic [15:0] o_out_w,
  output logic [15:0] o_out_h,

  // Puertos "base" de lectura (no usados como tal en el top actual,
  // pero se mantienen en la interfaz por compatibilidad)
  output logic [AW-1:0] in_raddr0,
  output logic [AW-1:0] in_raddr1,
  output logic [AW-1:0] in_raddr2,
  output logic [AW-1:0] in_raddr3,
  input  logic [7:0]    in_rdata0,
  input  logic [7:0]    in_rdata1,
  input  logic [7:0]    in_rdata2,
  input  logic [7:0]    in_rdata3,

  // ----------------- Puertos de dirección por lane --------------------------
  output logic [AW-1:0] in_raddr_lane0_0,
  output logic [AW-1:0] in_raddr_lane0_1,
  output logic [AW-1:0] in_raddr_lane0_2,
  output logic [AW-1:0] in_raddr_lane0_3,
  output logic [AW-1:0] in_raddr_lane1_0,
  output logic [AW-1:0] in_raddr_lane1_1,
  output logic [AW-1:0] in_raddr_lane1_2,
  output logic [AW-1:0] in_raddr_lane1_3,
  output logic [AW-1:0] in_raddr_lane2_0,
  output logic [AW-1:0] in_raddr_lane2_1,
  output logic [AW-1:0] in_raddr_lane2_2,
  output logic [AW-1:0] in_raddr_lane2_3,
  output logic [AW-1:0] in_raddr_lane3_0,
  output logic [AW-1:0] in_raddr_lane3_1,
  output logic [AW-1:0] in_raddr_lane3_2,
  output logic [AW-1:0] in_raddr_lane3_3,

  // ----------------- Puertos de datos por lane ------------------------------
  // En este diseño los datos efectivos se toman de lane0_* (banco mem_in),
  // pero los puertos de los demás lanes se conservan por compatibilidad.
  input  logic [7:0]    in_rdata_lane0_0,
  input  logic [7:0]    in_rdata_lane0_1,
  input  logic [7:0]    in_rdata_lane0_2,
  input  logic [7:0]    in_rdata_lane0_3,
  input  logic [7:0]    in_rdata_lane1_0,
  input  logic [7:0]    in_rdata_lane1_1,
  input  logic [7:0]    in_rdata_lane1_2,
  input  logic [7:0]    in_rdata_lane1_3,
  input  logic [7:0]    in_rdata_lane2_0,
  input  logic [7:0]    in_rdata_lane2_1,
  input  logic [7:0]    in_rdata_lane2_2,
  input  logic [7:0]    in_rdata_lane2_3,
  input  logic [7:0]    in_rdata_lane3_0,
  input  logic [7:0]    in_rdata_lane3_1,
  input  logic [7:0]    in_rdata_lane3_2,
  input  logic [7:0]    in_rdata_lane3_3,

  // Escrituras destino (4 puertos)
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
  output logic [31:0]  o_mem_rd_count,
  output logic [31:0]  o_mem_wr_count
);

  // ------------------------------------------------------------------------
  // Constantes
  // ------------------------------------------------------------------------
  localparam logic [8:0]  ONE_Q08    = 9'd256;        // 1.0 en Q0.8
  localparam logic [31:0] ROUND_Q016 = 32'h0000_8000; // para redondeo >>16
  localparam logic [31:0] FLOPS_PER_PIXEL = 32'd11;   // 8 mul + 3 sumas (aprox)

  // ------------------------------------------------------------------------
  // Dimensiones de salida y escala inversa
  // ------------------------------------------------------------------------
  logic [15:0] out_w_reg, out_h_reg;

  logic [31:0] mul_w, mul_h;
  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  logic [15:0] inv_scale_q88;

  function automatic logic [15:0] inv_q88(input logic [15:0] scale_q88);
    logic [31:0] num;
  begin
    if (scale_q88 == 16'd0) begin
      inv_q88 = 16'hFFFF;
    end else begin
      num     = 32'd65536;
      inv_q88 = num / scale_q88; // floor(65536 / scale_q88)
    end
  end
  endfunction

  logic [23:0] step_1x_q168;
  logic [23:0] step_4x_q168;
  assign step_1x_q168 = {8'd0, inv_scale_q88};
  assign step_4x_q168 = step_1x_q168 << 2; // 4 píxeles en X

  // ------------------------------------------------------------------------
  // Coordenadas de salida y fuente (Q16.8)
  // ------------------------------------------------------------------------
  logic [15:0] oy_cur;
  logic [15:0] group_x;
  logic [15:0] groups_per_row;

  logic [23:0] sy_fix_row;   // Q16.8
  logic [23:0] sx_fix_group; // Q16.8

  logic [23:0] sx_fix_lane [0:3];

  logic [15:0] sx_int_lane [0:3];
  logic [7:0]  ax_q_lane   [0:3];

  logic [15:0] sy_int_row;
  logic [7:0]  ay_q_row;

  assign sy_int_row = sy_fix_row[23:8];
  assign ay_q_row   = sy_fix_row[7:0];

  logic [15:0] xi_base_lane [0:3];
  logic [15:0] yi_base_row;
  logic [7:0]  fx_q_lane    [0:3];
  logic [7:0]  fy_q_row;

  // Intensidades de los vecinos
  logic [7:0] I00 [0:3];
  logic [7:0] I10 [0:3];
  logic [7:0] I01 [0:3];
  logic [7:0] I11 [0:3];

  // Pesos por lane
  logic [8:0] wx0_lane [0:3];
  logic [8:0] wx1_lane [0:3];
  logic [8:0] wy0_row;
  logic [8:0] wy1_row;

  logic [17:0] w00_q016_lane [0:3];
  logic [17:0] w10_q016_lane [0:3];
  logic [17:0] w01_q016_lane [0:3];
  logic [17:0] w11_q016_lane [0:3];

  // Productos registrados w*I en Q8.16
  logic [31:0] p00_r [0:3];
  logic [31:0] p10_r [0:3];
  logic [31:0] p01_r [0:3];
  logic [31:0] p11_r [0:3];

  // Suma bilineal y redondeo
  logic [31:0] sum_q016_lane    [0:3];
  logic [31:0] sum_rounded_lane [0:3];
  logic [7:0]  pix_lane         [0:3];

  // Coordenadas de salida por lane
  logic [15:0] ox_lane [0:3];

  // ------------------------------------------------------------------------
  // FSM
  // ------------------------------------------------------------------------
  typedef enum logic [4:0] {
    S_IDLE       = 5'd0,
    S_INIT       = 5'd1,
    S_ROW_INIT   = 5'd2,
    S_PIXEL_SETUP= 5'd3,
    S_READ00_A   = 5'd4,
    S_READ00_D   = 5'd5,
    S_READ10_A   = 5'd6,
    S_READ10_D   = 5'd7,
    S_READ01_A   = 5'd8,
    S_READ01_D   = 5'd9,
    S_READ11_A   = 5'd10,
    S_READ11_D   = 5'd11,
    S_MUL        = 5'd12,
    S_WRITE      = 5'd13,
    S_STEP_WAIT  = 5'd14,
    S_ADVANCE    = 5'd15,
    S_DONE       = 5'd16
  } state_t;

  state_t state, state_n;

  // ------------------------------------------------------------------------
  // Función para generar direcciones lineales
  // ------------------------------------------------------------------------
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

  // ------------------------------------------------------------------------
  // Pesos y suma (combinacional)
  // ------------------------------------------------------------------------
  assign wy0_row = ONE_Q08 - {1'b0, fy_q_row};
  assign wy1_row = {1'b0, fy_q_row};

  genvar gv;
  generate
    for (gv = 0; gv < 4; gv = gv + 1) begin : g_lane_ops
      assign wx0_lane[gv] = ONE_Q08 - {1'b0, fx_q_lane[gv]};
      assign wx1_lane[gv] = {1'b0, fx_q_lane[gv]};

      assign w00_q016_lane[gv] = wx0_lane[gv] * wy0_row;
      assign w10_q016_lane[gv] = wx1_lane[gv] * wy0_row;
      assign w01_q016_lane[gv] = wx0_lane[gv] * wy1_row;
      assign w11_q016_lane[gv] = wx1_lane[gv] * wy1_row;

      assign sum_q016_lane[gv]    = p00_r[gv] + p10_r[gv] + p01_r[gv] + p11_r[gv];
      assign sum_rounded_lane[gv] = sum_q016_lane[gv] + ROUND_Q016;
      assign pix_lane[gv]         = sum_rounded_lane[gv][23:16];
    end
  endgenerate

  // ------------------------------------------------------------------------
  // Alias de puertos de dirección a lane*_*
  //
  // Se reutilizan in_raddr0..3 como "base" y se replican a todos los lanes
  // de dirección para mantener compatibilidad de pines, pero en el top
  // actual los datos efectivos provienen de mem_in vía lane0_*.
// -------------------------------------------------------------------------
  always_comb begin
    // Direcciones base
    in_raddr_lane0_0 = in_raddr0;
    in_raddr_lane0_1 = in_raddr1;
    in_raddr_lane0_2 = in_raddr2;
    in_raddr_lane0_3 = in_raddr3;

    in_raddr_lane1_0 = in_raddr0;
    in_raddr_lane1_1 = in_raddr1;
    in_raddr_lane1_2 = in_raddr2;
    in_raddr_lane1_3 = in_raddr3;

    in_raddr_lane2_0 = in_raddr0;
    in_raddr_lane2_1 = in_raddr1;
    in_raddr_lane2_2 = in_raddr2;
    in_raddr_lane2_3 = in_raddr3;

    in_raddr_lane3_0 = in_raddr0;
    in_raddr_lane3_1 = in_raddr1;
    in_raddr_lane3_2 = in_raddr2;
    in_raddr_lane3_3 = in_raddr3;
  end

  // ------------------------------------------------------------------------
  // Secuencial principal
  // ------------------------------------------------------------------------
  integer i;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state       <= S_IDLE;
      busy        <= 1'b0;
      done        <= 1'b0;

      out_w_reg   <= 16'd0;
      out_h_reg   <= 16'd0;
      o_out_w     <= 16'd0;
      o_out_h     <= 16'd0;

      inv_scale_q88 <= 16'd0;

      oy_cur        <= 16'd0;
      group_x       <= 16'd0;
      groups_per_row<= 16'd0;

      sy_fix_row    <= 24'd0;
      sx_fix_group  <= 24'd0;

      yi_base_row   <= 16'd0;
      fy_q_row      <= 8'd0;

      for (i = 0; i < 4; i = i + 1) begin
        sx_fix_lane[i]   <= 24'd0;
        sx_int_lane[i]   <= 16'd0;
        ax_q_lane[i]     <= 8'd0;
        xi_base_lane[i]  <= 16'd0;
        fx_q_lane[i]     <= 8'd0;
        ox_lane[i]       <= 16'd0;

        I00[i]           <= 8'd0;
        I10[i]           <= 8'd0;
        I01[i]           <= 8'd0;
        I11[i]           <= 8'd0;

        p00_r[i]         <= 32'd0;
        p10_r[i]         <= 32'd0;
        p01_r[i]         <= 32'd0;
        p11_r[i]         <= 32'd0;
      end

      in_raddr0 <= '0;
      in_raddr1 <= '0;
      in_raddr2 <= '0;
      in_raddr3 <= '0;

      out_waddr0 <= '0; out_wdata0 <= 8'd0; out_we0 <= 1'b0;
      out_waddr1 <= '0; out_wdata1 <= 8'd0; out_we1 <= 1'b0;
      out_waddr2 <= '0; out_wdata2 <= 8'd0; out_we2 <= 1'b0;
      out_waddr3 <= '0; out_wdata3 <= 8'd0; out_we3 <= 1'b0;

      o_flop_count   <= 32'd0;
      o_mem_rd_count <= 32'd0;
      o_mem_wr_count <= 32'd0;

    end else begin
      state <= state_n;

      // Por defecto no escribir
      out_we0 <= 1'b0;
      out_we1 <= 1'b0;
      out_we2 <= 1'b0;
      out_we3 <= 1'b0;

      case (state)

        // ---------------------------------------------------
        // IDLE
        // ---------------------------------------------------
        S_IDLE: begin
          done <= 1'b0;
          busy <= 1'b0;
          if (start) begin
            // Dimensiones de salida igual al secuencial
            out_w_reg     <= mul_w[23:8];
            out_h_reg     <= mul_h[23:8];
            o_out_w       <= mul_w[23:8];
            o_out_h       <= mul_h[23:8];

            inv_scale_q88   <= inv_q88(i_scale_q88);
            groups_per_row  <= (mul_w[23:8] + 16'd3) >> 2;

            o_flop_count    <= 32'd0;
            o_mem_rd_count  <= 32'd0;
            o_mem_wr_count  <= 32'd0;
          end
        end

        // ---------------------------------------------------
        // INIT
        // ---------------------------------------------------
        S_INIT: begin
          busy         <= 1'b1;
          oy_cur       <= 16'd0;
          group_x      <= 16'd0;
          sy_fix_row   <= 24'd0;
          sx_fix_group <= 24'd0;
        end

        // ---------------------------------------------------
        // ROW_INIT
        // ---------------------------------------------------
        S_ROW_INIT: begin
          group_x      <= 16'd0;
          sx_fix_group <= 24'd0;
          // sy_fix_row ya fue actualizado en S_ADVANCE
        end

        // ---------------------------------------------------
        // PIXEL_SETUP: calcula coords para 4 lanes
        // ---------------------------------------------------
        S_PIXEL_SETUP: begin
          // sy_int_row / ay_q_row son derivados de sy_fix_row (Q16.8)
          // Clamp en Y
          logic [15:0] yi_tmp;
          logic [7:0]  fy_tmp;

          yi_tmp = sy_int_row;
          fy_tmp = ay_q_row;
          if (sy_int_row >= i_in_h - 16'd1) begin
            yi_tmp = i_in_h - 16'd2;
            fy_tmp = 8'hFF;
          end

          yi_base_row <= yi_tmp;
          fy_q_row    <= fy_tmp;

          // Lanes en X
          for (i = 0; i < 4; i = i + 1) begin
            logic [23:0] sx_fix_tmp;
            logic [15:0] xi_tmp;
            logic [7:0]  fx_tmp;

            ox_lane[i] <= (group_x << 2) + i[15:0];

            // sx_fix_group es la base para el grupo; cada lane avanza 1 step
            sx_fix_tmp = sx_fix_group + (step_1x_q168 * i[3:0]);
            sx_fix_lane[i] <= sx_fix_tmp;

            xi_tmp = sx_fix_tmp[23:8];
            fx_tmp = sx_fix_tmp[7:0];

            // Clamp en X
            if (xi_tmp >= i_in_w - 16'd1) begin
              xi_tmp = i_in_w - 16'd2;
              fx_tmp = 8'hFF;
            end

            sx_int_lane[i]  <= xi_tmp;  // parte entera de la fuente en X
            ax_q_lane[i]    <= fx_tmp;  // fracción en X
            fx_q_lane[i]    <= fx_tmp;
            xi_base_lane[i] <= xi_tmp;
          end

          // En el siguiente estado (S_READ00_A) se ponen las direcciones
          // de I00.
        end

        // ---------------------------------------------------
        // Lectura I00: dirección
        // ---------------------------------------------------
        S_READ00_A: begin
          in_raddr0 <= linaddr(xi_base_lane[0], yi_base_row, i_in_w);
          in_raddr1 <= linaddr(xi_base_lane[1], yi_base_row, i_in_w);
          in_raddr2 <= linaddr(xi_base_lane[2], yi_base_row, i_in_w);
          in_raddr3 <= linaddr(xi_base_lane[3], yi_base_row, i_in_w);
        end

        // Lectura I00: datos (DESDE lane0, que viene de mem_in en el top)
        S_READ00_D: begin
          I00[0] <= in_rdata_lane0_0;
          I00[1] <= in_rdata_lane0_1;
          I00[2] <= in_rdata_lane0_2;
          I00[3] <= in_rdata_lane0_3;

          o_mem_rd_count <= o_mem_rd_count + 32'd4;
        end

        // ---------------------------------------------------
        // Lectura I10
        // ---------------------------------------------------
        S_READ10_A: begin
          in_raddr0 <= linaddr(xi_base_lane[0] + 16'd1, yi_base_row, i_in_w);
          in_raddr1 <= linaddr(xi_base_lane[1] + 16'd1, yi_base_row, i_in_w);
          in_raddr2 <= linaddr(xi_base_lane[2] + 16'd1, yi_base_row, i_in_w);
          in_raddr3 <= linaddr(xi_base_lane[3] + 16'd1, yi_base_row, i_in_w);
        end

        S_READ10_D: begin
          I10[0] <= in_rdata_lane0_0;
          I10[1] <= in_rdata_lane0_1;
          I10[2] <= in_rdata_lane0_2;
          I10[3] <= in_rdata_lane0_3;

          o_mem_rd_count <= o_mem_rd_count + 32'd4;
        end

        // ---------------------------------------------------
        // Lectura I01
        // ---------------------------------------------------
        S_READ01_A: begin
          in_raddr0 <= linaddr(xi_base_lane[0], yi_base_row + 16'd1, i_in_w);
          in_raddr1 <= linaddr(xi_base_lane[1], yi_base_row + 16'd1, i_in_w);
          in_raddr2 <= linaddr(xi_base_lane[2], yi_base_row + 16'd1, i_in_w);
          in_raddr3 <= linaddr(xi_base_lane[3], yi_base_row + 16'd1, i_in_w);
        end

        S_READ01_D: begin
          I01[0] <= in_rdata_lane0_0;
          I01[1] <= in_rdata_lane0_1;
          I01[2] <= in_rdata_lane0_2;
          I01[3] <= in_rdata_lane0_3;

          o_mem_rd_count <= o_mem_rd_count + 32'd4;
        end

        // ---------------------------------------------------
        // Lectura I11
        // ---------------------------------------------------
        S_READ11_A: begin
          in_raddr0 <= linaddr(xi_base_lane[0] + 16'd1, yi_base_row + 16'd1, i_in_w);
          in_raddr1 <= linaddr(xi_base_lane[1] + 16'd1, yi_base_row + 16'd1, i_in_w);
          in_raddr2 <= linaddr(xi_base_lane[2] + 16'd1, yi_base_row + 16'd1, i_in_w);
          in_raddr3 <= linaddr(xi_base_lane[3] + 16'd1, yi_base_row + 16'd1, i_in_w);
        end

        S_READ11_D: begin
          I11[0] <= in_rdata_lane0_0;
          I11[1] <= in_rdata_lane0_1;
          I11[2] <= in_rdata_lane0_2;
          I11[3] <= in_rdata_lane0_3;

          o_mem_rd_count <= o_mem_rd_count + 32'd4;
        end

        // ---------------------------------------------------
        // Productos w*I (Q8.16)
        // ---------------------------------------------------
        S_MUL: begin
          for (i = 0; i < 4; i = i + 1) begin
            p00_r[i] <= w00_q016_lane[i] * I00[i];
            p10_r[i] <= w10_q016_lane[i] * I10[i];
            p01_r[i] <= w01_q016_lane[i] * I01[i];
            p11_r[i] <= w11_q016_lane[i] * I11[i];
          end
        end

        // ---------------------------------------------------
        // Escritura de 4 píxeles (si están dentro de out_w)
        // ---------------------------------------------------
        S_WRITE: begin
          integer n_wr;
          integer n_pix;
          n_wr  = 0;
          n_pix = 0;

          // lane 0
          if (ox_lane[0] < out_w_reg) begin
            out_waddr0 <= linaddr(ox_lane[0], oy_cur, out_w_reg);
            out_wdata0 <= pix_lane[0];
            out_we0    <= 1'b1;
            n_wr      += 1;
            n_pix     += 1;
          end

          // lane 1
          if (ox_lane[1] < out_w_reg) begin
            out_waddr1 <= linaddr(ox_lane[1], oy_cur, out_w_reg);
            out_wdata1 <= pix_lane[1];
            out_we1    <= 1'b1;
            n_wr      += 1;
            n_pix     += 1;
          end

          // lane 2
          if (ox_lane[2] < out_w_reg) begin
            out_waddr2 <= linaddr(ox_lane[2], oy_cur, out_w_reg);
            out_wdata2 <= pix_lane[2];
            out_we2    <= 1'b1;
            n_wr      += 1;
            n_pix     += 1;
          end

          // lane 3
          if (ox_lane[3] < out_w_reg) begin
            out_waddr3 <= linaddr(ox_lane[3], oy_cur, out_w_reg);
            out_wdata3 <= pix_lane[3];
            out_we3    <= 1'b1;
            n_wr      += 1;
            n_pix     += 1;
          end

          o_mem_wr_count <= o_mem_wr_count + n_wr;
          o_flop_count   <= o_flop_count   + (n_pix * FLOPS_PER_PIXEL);
        end

        // ---------------------------------------------------
        // STEP_WAIT: modo stepping
        // ---------------------------------------------------
        S_STEP_WAIT: begin
          // la transición se maneja en la lógica de próximo estado
        end

        // ---------------------------------------------------
        // ADVANCE: siguiente grupo o siguiente fila
        // ---------------------------------------------------
        S_ADVANCE: begin
          if (group_x + 16'd1 < groups_per_row) begin
            group_x      <= group_x + 16'd1;
            sx_fix_group <= sx_fix_group + step_4x_q168;
          end else begin
            if (oy_cur + 16'd1 < out_h_reg) begin
              oy_cur     <= oy_cur + 16'd1;
              sy_fix_row <= sy_fix_row + step_1x_q168;
              // sx_fix_group se reinicia en ROW_INIT
            end
          end
        end

        // ---------------------------------------------------
        // DONE
        // ---------------------------------------------------
        S_DONE: begin
          busy <= 1'b0;
          done <= 1'b1;
        end

      endcase
    end
  end

  // ------------------------------------------------------------------------
  // Próximo estado
  // ------------------------------------------------------------------------
  always_comb begin
    state_n = state;

    unique case (state)
      S_IDLE: begin
        if (start) state_n = S_INIT;
      end

      S_INIT:       state_n = S_ROW_INIT;
      S_ROW_INIT:   state_n = S_PIXEL_SETUP;
      S_PIXEL_SETUP:state_n = S_READ00_A;

      S_READ00_A:   state_n = S_READ00_D;
      S_READ00_D:   state_n = S_READ10_A;
      S_READ10_A:   state_n = S_READ10_D;
      S_READ10_D:   state_n = S_READ01_A;
      S_READ01_A:   state_n = S_READ01_D;
      S_READ01_D:   state_n = S_READ11_A;
      S_READ11_A:   state_n = S_READ11_D;
      S_READ11_D:   state_n = S_MUL;
      S_MUL:        state_n = S_WRITE;

      S_WRITE: begin
        if (i_step_en) begin
          state_n = S_STEP_WAIT;
        end else begin
          if ((group_x + 16'd1 >= groups_per_row) &&
              (oy_cur   + 16'd1 >= out_h_reg))
            state_n = S_DONE;
          else
            state_n = S_ADVANCE;
        end
      end

      S_STEP_WAIT: begin
        if (i_step_pulse) begin
          if ((group_x + 16'd1 >= groups_per_row) &&
              (oy_cur   + 16'd1 >= out_h_reg))
            state_n = S_DONE;
          else
            state_n = S_ADVANCE;
        end
      end

      S_ADVANCE: begin
        if ((group_x + 16'd1 >= groups_per_row) &&
            (oy_cur   + 16'd1 >= out_h_reg))
          state_n = S_DONE;
        else if (group_x + 16'd1 >= groups_per_row)
          state_n = S_ROW_INIT;
        else
          state_n = S_PIXEL_SETUP;
      end

      S_DONE: state_n = S_IDLE;

      default: state_n = S_IDLE;
    endcase
  end

endmodule
