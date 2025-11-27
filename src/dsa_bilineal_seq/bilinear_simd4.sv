`timescale 1ns/1ps

// ============================================================================
// bilinear_simd4.sv  — Núcleo SIMD x4 para interpolación bilineal Q8.8.
//
// - Misma interpolación que bilinear_seq (Q8.8).
// - Procesa hasta 4 píxeles por grupo (SIMD x4).
// - FSM optimizada: lectura de los 16 vecinos (4 píxeles × 4 vecinos)
//   en un único par de estados READ_A / READ_D (2 ciclos de lectura).
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

  // ----------------- Puertos de dirección por lane --------------------------
  // Banco 0: mem_in
  output logic [AW-1:0] in_raddr_lane0_0, // I00 lane0
  output logic [AW-1:0] in_raddr_lane0_1, // I10 lane0
  output logic [AW-1:0] in_raddr_lane0_2, // I01 lane0
  output logic [AW-1:0] in_raddr_lane0_3, // I11 lane0
  // Banco 1: mem_in1
  output logic [AW-1:0] in_raddr_lane1_0,
  output logic [AW-1:0] in_raddr_lane1_1,
  output logic [AW-1:0] in_raddr_lane1_2,
  output logic [AW-1:0] in_raddr_lane1_3,
  // Banco 2: mem_in2
  output logic [AW-1:0] in_raddr_lane2_0,
  output logic [AW-1:0] in_raddr_lane2_1,
  output logic [AW-1:0] in_raddr_lane2_2,
  output logic [AW-1:0] in_raddr_lane2_3,
  // Banco 3: mem_in3
  output logic [AW-1:0] in_raddr_lane3_0,
  output logic [AW-1:0] in_raddr_lane3_1,
  output logic [AW-1:0] in_raddr_lane3_2,
  output logic [AW-1:0] in_raddr_lane3_3,

  // ----------------- Puertos de datos por lane ------------------------------
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
  localparam logic [8:0]  ONE_Q08          = 9'd256;        // 1.0 en Q0.8
  localparam logic [31:0] ROUND_Q016       = 32'h0000_8000; // para redondeo >>16
  localparam logic [31:0] FLOPS_PER_PIXEL  = 32'd11;        // aprox 8 mul + 3 sumas

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
      num     = 32'd65536;         // 256*256
      inv_q88 = (num / scale_q88); // floor(65536 / scale_q88)
    end
  end
  endfunction

  // ------------------------------------------------------------------------
  // Coordenadas de salida y fuente (Q16.8)
  // ------------------------------------------------------------------------
  logic [15:0] oy_cur;
  logic [15:0] group_x;
  logic [15:0] groups_per_row;

  // Q16.8 en Y (por fila)
  logic [23:0] sy_fix_row;

  // Pasos en Q16.8
  logic [23:0] inv_step_q168;   // = {8'd0, inv_scale_q88}
  logic [23:0] four_step_q168;  // = 4*inv_step_q168

  assign inv_step_q168  = {8'd0, inv_scale_q88};
  assign four_step_q168 = inv_step_q168 << 2;

  // Q16.8 por grupo en X
  logic [23:0] sx_fix_group;
  // Temporal para calcular sx por lane
  logic [23:0] sx_loc;

  // Partes enteras y fracc. por lane (fuente base/fracción)
  logic [15:0] xi_base_lane [0:3];
  logic [15:0] yi_base_row;
  logic [7:0]  fx_q_lane    [0:3];
  logic [7:0]  fy_q_row;

  // Temps de clamping X/Y
  logic [15:0] yi_tmp;
  logic [7:0]  fy_tmp;
  logic [15:0] xi_tmp_lane [0:3];
  logic [7:0]  fx_tmp_lane [0:3];

  // Coordenadas de salida
  logic [15:0] ox_lane    [0:3];
  logic [15:0] ox_tmp_lane[0:3];

  // Intensidades fuente por lane
  logic [7:0] I00 [0:3];
  logic [7:0] I10 [0:3];
  logic [7:0] I01 [0:3];
  logic [7:0] I11 [0:3];

  // Pesos
  logic [8:0] wx0_lane [0:3];
  logic [8:0] wx1_lane [0:3];
  logic [8:0] wy0_row;
  logic [8:0] wy1_row;

  logic [17:0] w00_q016_lane [0:3];
  logic [17:0] w10_q016_lane [0:3];
  logic [17:0] w01_q016_lane [0:3];
  logic [17:0] w11_q016_lane [0:3];

  // Productos registrados Q8.16
  logic [31:0] p00_r [0:3];
  logic [31:0] p10_r [0:3];
  logic [31:0] p01_r [0:3];
  logic [31:0] p11_r [0:3];

  // Suma bilineal
  logic [31:0] sum_q016_lane    [0:3];
  logic [31:0] sum_rounded_lane [0:3];
  logic [7:0]  pix_lane         [0:3];

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
  // FSM
  // ------------------------------------------------------------------------
  typedef enum logic [4:0] {
    S_IDLE        = 5'd0,
    S_INIT        = 5'd1,
    S_ROW_INIT    = 5'd2,
    S_PIXEL_SETUP = 5'd3,
    S_READ_A      = 5'd4,
    S_READ_D      = 5'd5,
    S_MUL         = 5'd6,
    S_WRITE       = 5'd7,
    S_STEP_WAIT   = 5'd8,
    S_ADVANCE     = 5'd9,
    S_DONE        = 5'd10
  } state_t;

  state_t state, state_n;

  // ------------------------------------------------------------------------
  // linaddr
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
  // Secuencial principal
  // ------------------------------------------------------------------------
  integer i;
  integer n_wr;
  integer n_pix;

  // Partes enteras/fracción de sy_fix_row
  logic [15:0] sy_int_row;
  logic [7:0]  ay_q_row;

  assign sy_int_row = sy_fix_row[23:8];
  assign ay_q_row   = sy_fix_row[7:0];

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
      sx_loc        <= 24'd0;

      yi_base_row   <= 16'd0;
      fy_q_row      <= 8'd0;
      yi_tmp        <= 16'd0;
      fy_tmp        <= 8'd0;

      for (i = 0; i < 4; i = i + 1) begin
        xi_base_lane[i]  <= 16'd0;
        fx_q_lane[i]     <= 8'd0;
        xi_tmp_lane[i]   <= 16'd0;
        fx_tmp_lane[i]   <= 8'd0;

        ox_lane[i]       <= 16'd0;
        ox_tmp_lane[i]   <= 16'd0;

        I00[i]           <= 8'd0;
        I10[i]           <= 8'd0;
        I01[i]           <= 8'd0;
        I11[i]           <= 8'd0;

        p00_r[i]         <= 32'd0;
        p10_r[i]         <= 32'd0;
        p01_r[i]         <= 32'd0;
        p11_r[i]         <= 32'd0;
      end

      in_raddr_lane0_0 <= '0;
      in_raddr_lane0_1 <= '0;
      in_raddr_lane0_2 <= '0;
      in_raddr_lane0_3 <= '0;
      in_raddr_lane1_0 <= '0;
      in_raddr_lane1_1 <= '0;
      in_raddr_lane1_2 <= '0;
      in_raddr_lane1_3 <= '0;
      in_raddr_lane2_0 <= '0;
      in_raddr_lane2_1 <= '0;
      in_raddr_lane2_2 <= '0;
      in_raddr_lane2_3 <= '0;
      in_raddr_lane3_0 <= '0;
      in_raddr_lane3_1 <= '0;
      in_raddr_lane3_2 <= '0;
      in_raddr_lane3_3 <= '0;

      out_waddr0 <= '0; out_wdata0 <= 8'd0; out_we0 <= 1'b0;
      out_waddr1 <= '0; out_wdata1 <= 8'd0; out_we1 <= 1'b0;
      out_waddr2 <= '0; out_wdata2 <= 8'd0; out_we2 <= 1'b0;
      out_waddr3 <= '0; out_wdata3 <= 8'd0; out_we3 <= 1'b0;

      o_flop_count   <= 32'd0;
      o_mem_rd_count <= 32'd0;
      o_mem_wr_count <= 32'd0;

      n_wr  <= 0;
      n_pix <= 0;

    end else begin
      state <= state_n;

      // Por defecto sin escritura
      out_we0 <= 1'b0;
      out_we1 <= 1'b0;
      out_we2 <= 1'b0;
      out_we3 <= 1'b0;

      case (state)

        // ------------------ IDLE ---------------------------
        S_IDLE: begin
          done <= 1'b0;
          busy <= 1'b0;
          if (start) begin
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

        // ------------------ INIT --------------------------
        S_INIT: begin
          busy        <= 1'b1;
          oy_cur      <= 16'd0;
          group_x     <= 16'd0;

          sy_fix_row  <= 24'd0;  // fila 0
          sx_fix_group<= 24'd0;  // grupo 0
        end

        // ------------------ ROW_INIT ----------------------
        S_ROW_INIT: begin
          group_x      <= 16'd0;
          sx_fix_group <= 24'd0;
        end

        // ------------------ PIXEL_SETUP -------------------
        S_PIXEL_SETUP: begin
          // Y desde sy_fix_row (Q16.8 acumulada)
          yi_tmp = sy_int_row;
          fy_tmp = ay_q_row;

          if (sy_int_row >= i_in_h - 16'd1) begin
            yi_tmp = i_in_h - 16'd2;
            fy_tmp = 8'hFF;
          end

          yi_base_row <= yi_tmp;
          fy_q_row    <= fy_tmp;

          // X por lane
          for (i = 0; i < 4; i = i + 1) begin
            ox_tmp_lane[i] = (group_x << 2) + i[15:0];
            ox_lane[i]     <= ox_tmp_lane[i];

            case (i)
              0: sx_loc = sx_fix_group;
              1: sx_loc = sx_fix_group + inv_step_q168;
              2: sx_loc = sx_fix_group + (inv_step_q168 << 1);
              3: sx_loc = sx_fix_group + inv_step_q168 + (inv_step_q168 << 1);
              default: sx_loc = sx_fix_group;
            endcase

            xi_tmp_lane[i] = sx_loc[23:8];
            fx_tmp_lane[i] = sx_loc[7:0];

            if (xi_tmp_lane[i] >= i_in_w - 16'd1) begin
              xi_tmp_lane[i] = i_in_w - 16'd2;
              fx_tmp_lane[i] = 8'hFF;
            end

            fx_q_lane[i]    <= fx_tmp_lane[i];
            xi_base_lane[i] <= xi_tmp_lane[i];
          end
        end

        // ------------------ READ (direcciones) ------------
        S_READ_A: begin
          // Lane 0
          in_raddr_lane0_0 <= linaddr(xi_base_lane[0],           yi_base_row,         i_in_w);
          in_raddr_lane0_1 <= linaddr(xi_base_lane[0] + 16'd1,   yi_base_row,         i_in_w);
          in_raddr_lane0_2 <= linaddr(xi_base_lane[0],           yi_base_row + 16'd1, i_in_w);
          in_raddr_lane0_3 <= linaddr(xi_base_lane[0] + 16'd1,   yi_base_row + 16'd1, i_in_w);

          // Lane 1
          in_raddr_lane1_0 <= linaddr(xi_base_lane[1],           yi_base_row,         i_in_w);
          in_raddr_lane1_1 <= linaddr(xi_base_lane[1] + 16'd1,   yi_base_row,         i_in_w);
          in_raddr_lane1_2 <= linaddr(xi_base_lane[1],           yi_base_row + 16'd1, i_in_w);
          in_raddr_lane1_3 <= linaddr(xi_base_lane[1] + 16'd1,   yi_base_row + 16'd1, i_in_w);

          // Lane 2
          in_raddr_lane2_0 <= linaddr(xi_base_lane[2],           yi_base_row,         i_in_w);
          in_raddr_lane2_1 <= linaddr(xi_base_lane[2] + 16'd1,   yi_base_row,         i_in_w);
          in_raddr_lane2_2 <= linaddr(xi_base_lane[2],           yi_base_row + 16'd1, i_in_w);
          in_raddr_lane2_3 <= linaddr(xi_base_lane[2] + 16'd1,   yi_base_row + 16'd1, i_in_w);

          // Lane 3
          in_raddr_lane3_0 <= linaddr(xi_base_lane[3],           yi_base_row,         i_in_w);
          in_raddr_lane3_1 <= linaddr(xi_base_lane[3] + 16'd1,   yi_base_row,         i_in_w);
          in_raddr_lane3_2 <= linaddr(xi_base_lane[3],           yi_base_row + 16'd1, i_in_w);
          in_raddr_lane3_3 <= linaddr(xi_base_lane[3] + 16'd1,   yi_base_row + 16'd1, i_in_w);
        end

        // ------------------ READ (datos) ------------------
        S_READ_D: begin
          // Lane 0
          I00[0] <= in_rdata_lane0_0;
          I10[0] <= in_rdata_lane0_1;
          I01[0] <= in_rdata_lane0_2;
          I11[0] <= in_rdata_lane0_3;

          // Lane 1
          I00[1] <= in_rdata_lane1_0;
          I10[1] <= in_rdata_lane1_1;
          I01[1] <= in_rdata_lane1_2;
          I11[1] <= in_rdata_lane1_3;

          // Lane 2
          I00[2] <= in_rdata_lane2_0;
          I10[2] <= in_rdata_lane2_1;
          I01[2] <= in_rdata_lane2_2;
          I11[2] <= in_rdata_lane2_3;

          // Lane 3
          I00[3] <= in_rdata_lane3_0;
          I10[3] <= in_rdata_lane3_1;
          I01[3] <= in_rdata_lane3_2;
          I11[3] <= in_rdata_lane3_3;

          o_mem_rd_count <= o_mem_rd_count + 32'd16;
        end

        // ------------------ MUL ---------------------------
        S_MUL: begin
          for (i = 0; i < 4; i = i + 1) begin
            p00_r[i] <= w00_q016_lane[i] * I00[i];
            p10_r[i] <= w10_q016_lane[i] * I10[i];
            p01_r[i] <= w01_q016_lane[i] * I01[i];
            p11_r[i] <= w11_q016_lane[i] * I11[i];
          end
        end

        // ------------------ WRITE -------------------------
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

        // ------------------ STEP_WAIT ---------------------
        S_STEP_WAIT: begin
          // Avance controlado por i_step_pulse en state_n
        end

        // ------------------ ADVANCE -----------------------
        S_ADVANCE: begin
          if (group_x + 16'd1 < groups_per_row) begin
            group_x      <= group_x + 16'd1;
            sx_fix_group <= sx_fix_group + four_step_q168;
          end else begin
            if (oy_cur + 16'd1 < out_h_reg) begin
              oy_cur      <= oy_cur + 16'd1;
              sy_fix_row  <= sy_fix_row + inv_step_q168;
              group_x     <= 16'd0;
              sx_fix_group<= 24'd0;
            end
          end
        end

        // ------------------ DONE --------------------------
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
      S_IDLE:        if (start) state_n = S_INIT;
      S_INIT:        state_n = S_ROW_INIT;
      S_ROW_INIT:    state_n = S_PIXEL_SETUP;
      S_PIXEL_SETUP: state_n = S_READ_A;
      S_READ_A:      state_n = S_READ_D;
      S_READ_D:      state_n = S_MUL;
      S_MUL:         state_n = S_WRITE;

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
