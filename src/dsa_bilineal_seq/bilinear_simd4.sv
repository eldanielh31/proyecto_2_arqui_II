`timescale 1ns/1ps

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

  localparam int LANES = 4;

  localparam logic [8:0]  ONE_Q08         = 9'd256;
  localparam logic [31:0] FLOPS_PER_PIXEL = 32'd11;
  localparam logic [31:0] ROUND_Q016      = 32'h0000_8000;

  // Dimensiones de salida internas
  logic [15:0] out_w_reg, out_h_reg;

  // Coordenadas de salida
  logic [15:0] oy_cur;          // fila de salida actual
  logic [15:0] group_ox;        // x base del grupo (0,4,8,...)

  // Coordenadas x de salida por lane
  logic [15:0] lane_x    [LANES];
  logic        lane_valid[LANES];

  // Inversa de escala
  logic [15:0] inv_scale_q88;

  // Coordenadas fuente por lane
  logic [15:0] xi_base [LANES];
  logic [7:0]  fx_q    [LANES];

  // Coordenadas fuente en Y (comunes a todos los lanes)
  logic [23:0] sy_fix;
  logic [15:0] yi_base_row;
  logic [7:0]  fy_q_row;

  // Intensidades
  logic [7:0] I00[LANES];
  logic [7:0] I10[LANES];
  logic [7:0] I01[LANES];
  logic [7:0] I11[LANES];

  // Pesos
  logic [8:0] wx0_ext [LANES];
  logic [8:0] wx1_ext [LANES];
  logic [8:0] wy0_ext;
  logic [8:0] wy1_ext;

  logic [17:0] w00_q016[LANES];
  logic [17:0] w10_q016[LANES];
  logic [17:0] w01_q016[LANES];
  logic [17:0] w11_q016[LANES];

  // Productos
  logic [31:0] p00_r[LANES];
  logic [31:0] p10_r[LANES];
  logic [31:0] p01_r[LANES];
  logic [31:0] p11_r[LANES];

  logic [31:0] sum_q016_lane    [LANES];
  logic [31:0] sum_rounded_lane [LANES];
  logic [7:0]  PIX_lane         [LANES];

  // Multiplicaciones W*scale / H*scale (combinacional)
  logic [31:0] mul_w, mul_h;
  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  // ---------------------------------------------------------------
  // Funciones auxiliares
  // ---------------------------------------------------------------
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

  function automatic logic [15:0] inv_q88(input logic [15:0] scale_q88);
    logic [31:0] num;
    logic [31:0] quo;
  begin
    if (scale_q88 == 16'd0) begin
      inv_q88 = 16'hFFFF;
    end else begin
      num = 32'd65536 + (scale_q88 >> 8);
      quo = num / scale_q88;
      inv_q88 = quo[15:0];
    end
  end
  endfunction

  // ---------------------------------------------------------------
  // Pesos y suma por lane (combinacional)
  // ---------------------------------------------------------------
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

  assign wy0_ext = ONE_Q08 - {1'b0, fy_q_row};
  assign wy1_ext = {1'b0, fy_q_row};

  // FLOPs por grupo (4 lanes)
  logic [31:0] flops_incr;
  integer k_fl;
  always_comb begin
    flops_incr = 32'd0;
    for (k_fl = 0; k_fl < LANES; k_fl = k_fl + 1)
      if (lane_valid[k_fl])
        flops_incr = flops_incr + FLOPS_PER_PIXEL;
  end

  // ---------------------------------------------------------------
  // FSM
  // ---------------------------------------------------------------
  typedef enum logic [4:0] {
    S_IDLE        = 5'd0,
    S_INIT        = 5'd1,
    S_ROW_INIT    = 5'd2,
    S_GROUP_SETUP = 5'd3,
    S_G00_PRE     = 5'd4,
    S_G00         = 5'd5,
    S_G10_PRE     = 5'd6,
    S_G10         = 5'd7,
    S_G01_PRE     = 5'd8,
    S_G01         = 5'd9,
    S_G11_PRE     = 5'd10,
    S_G11         = 5'd11,
    S_MUL         = 5'd12,
    S_WRITE0      = 5'd13,
    S_WRITE1      = 5'd14,
    S_WRITE2      = 5'd15,
    S_WRITE3      = 5'd16,
    S_NEXT_GROUP  = 5'd17,
    S_NEXT_ROW    = 5'd18,
    S_DONE        = 5'd19
  } state_t;

  state_t state, state_n;
  logic [1:0] lane_idx;

  // Próximo estado
  always_comb begin
    state_n = state;
    unique case (state)
      S_IDLE: begin
        if (start) state_n = S_INIT;
      end

      S_INIT: begin
        state_n = S_ROW_INIT;
      end

      S_ROW_INIT: begin
        state_n = S_GROUP_SETUP;
      end

      S_GROUP_SETUP: begin
        // Siempre procesar el grupo; las lanes inválidas se filtran con lane_valid
        state_n = S_G00_PRE;
      end

      S_G00_PRE: state_n = S_G00;

      S_G00: begin
        if (lane_idx == 2'd3) state_n = S_G10_PRE;
      end

      S_G10_PRE: state_n = S_G10;

      S_G10: begin
        if (lane_idx == 2'd3) state_n = S_G01_PRE;
      end

      S_G01_PRE: state_n = S_G01;

      S_G01: begin
        if (lane_idx == 2'd3) state_n = S_G11_PRE;
      end

      S_G11_PRE: state_n = S_G11;

      S_G11: begin
        if (lane_idx == 2'd3) state_n = S_MUL;
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

  // Variables temporales
  logic [31:0] sy_mul;
  logic [23:0] sy_next;
  logic [15:0] sy_int_next;
  logic [7:0]  ay_next;

  logic [31:0] sx_mul;
  logic [23:0] sx_fix_tmp;
  logic [15:0] sx_int_tmp;
  logic [7:0]  ax_tmp;

  integer i;

  // ---------------------------------------------------------------
  // Lógica secuencial
  // ---------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state   <= S_IDLE;
      busy    <= 1'b0;
      done    <= 1'b0;
      out_we  <= 1'b0;

      in_raddr   <= '0;
      out_waddr  <= '0;
      out_wdata  <= 8'h00;

      out_w_reg  <= 16'd0;
      out_h_reg  <= 16'd0;
      o_out_w    <= 16'd0;
      o_out_h    <= 16'd0;
      inv_scale_q88 <= 16'd0;

      oy_cur     <= 16'd0;
      group_ox   <= 16'd0;
      sy_fix     <= 24'd0;
      yi_base_row<= 16'd0;
      fy_q_row   <= 8'd0;

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
      out_we <= 1'b0;

      unique case (state)
        // ---------------------------------------------------------
        S_IDLE: begin
          busy <= 1'b0;
          done <= 1'b0;
          if (start) begin
            // Nada aquí: se hace en S_INIT para evitar carreras
          end
        end

        // ---------------------------------------------------------
        S_INIT: begin
          busy          <= 1'b1;
          // Dimensiones de salida y escala inversa igual que en seq
          out_w_reg     <= mul_w[23:8];
          out_h_reg     <= mul_h[23:8];
          o_out_w       <= mul_w[23:8];
          o_out_h       <= mul_h[23:8];
          inv_scale_q88 <= inv_q88(i_scale_q88);

          oy_cur        <= 16'd0;
          group_ox      <= 16'd0;
          sy_fix        <= 24'd0;

          o_flop_count   <= 32'd0;
          o_mem_rd_count <= 32'd0;
          o_mem_wr_count <= 32'd0;
        end

        // ---------------------------------------------------------
        S_ROW_INIT: begin
          group_ox <= 16'd0;

          sy_mul      = oy_cur * inv_scale_q88;
          sy_next     = sy_mul[23:0];
          sy_int_next = sy_next[23:8];
          ay_next     = sy_next[7:0];

          sy_fix <= sy_next;

          if (sy_int_next >= i_in_h - 16'd1) begin
            yi_base_row <= i_in_h - 16'd2;
            fy_q_row    <= 8'hFF;
          end else begin
            yi_base_row <= sy_int_next;
            fy_q_row    <= ay_next;
          end
        end

        // ---------------------------------------------------------
        S_GROUP_SETUP: begin
          for (i = 0; i < LANES; i = i + 1) begin
            lane_x[i]     <= group_ox + i[15:0];
            lane_valid[i] <= ((group_ox + i[15:0]) < out_w_reg);

            sx_mul     = (group_ox + i[15:0]) * inv_scale_q88;
            sx_fix_tmp = sx_mul[23:0];
            sx_int_tmp = sx_fix_tmp[23:8];
            ax_tmp     = sx_fix_tmp[7:0];

            if (sx_int_tmp >= i_in_w - 16'd1) begin
              xi_base[i] <= i_in_w - 16'd2;
              fx_q[i]    <= 8'hFF;
            end else begin
              xi_base[i] <= sx_int_tmp;
              fx_q[i]    <= ax_tmp;
            end
          end
        end

        // ---------------------------------------------------------
        // GATHER I00
        // ---------------------------------------------------------
        S_G00_PRE: begin
          lane_idx <= 2'd0;
          in_raddr <= linaddr(xi_base[0], yi_base_row, i_in_w);
        end

        S_G00: begin
          case (lane_idx)
            2'd0: begin
              if (lane_valid[0]) begin
                I00[0]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd1;
              in_raddr <= linaddr(xi_base[1], yi_base_row, i_in_w);
            end
            2'd1: begin
              if (lane_valid[1]) begin
                I00[1]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd2;
              in_raddr <= linaddr(xi_base[2], yi_base_row, i_in_w);
            end
            2'd2: begin
              if (lane_valid[2]) begin
                I00[2]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd3;
              in_raddr <= linaddr(xi_base[3], yi_base_row, i_in_w);
            end
            2'd3: begin
              if (lane_valid[3]) begin
                I00[3]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              in_raddr <= '0;
            end
          endcase
        end

        // ---------------------------------------------------------
        // GATHER I10
        // ---------------------------------------------------------
        S_G10_PRE: begin
          lane_idx <= 2'd0;
          in_raddr <= linaddr(xi_base[0] + 16'd1, yi_base_row, i_in_w);
        end

        S_G10: begin
          case (lane_idx)
            2'd0: begin
              if (lane_valid[0]) begin
                I10[0]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd1;
              in_raddr <= linaddr(xi_base[1] + 16'd1, yi_base_row, i_in_w);
            end
            2'd1: begin
              if (lane_valid[1]) begin
                I10[1]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd2;
              in_raddr <= linaddr(xi_base[2] + 16'd1, yi_base_row, i_in_w);
            end
            2'd2: begin
              if (lane_valid[2]) begin
                I10[2]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd3;
              in_raddr <= linaddr(xi_base[3] + 16'd1, yi_base_row, i_in_w);
            end
            2'd3: begin
              if (lane_valid[3]) begin
                I10[3]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              in_raddr <= '0;
            end
          endcase
        end

        // ---------------------------------------------------------
        // GATHER I01
        // ---------------------------------------------------------
        S_G01_PRE: begin
          lane_idx <= 2'd0;
          in_raddr <= linaddr(xi_base[0], yi_base_row + 16'd1, i_in_w);
        end

        S_G01: begin
          case (lane_idx)
            2'd0: begin
              if (lane_valid[0]) begin
                I01[0]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd1;
              in_raddr <= linaddr(xi_base[1], yi_base_row + 16'd1, i_in_w);
            end
            2'd1: begin
              if (lane_valid[1]) begin
                I01[1]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd2;
              in_raddr <= linaddr(xi_base[2], yi_base_row + 16'd1, i_in_w);
            end
            2'd2: begin
              if (lane_valid[2]) begin
                I01[2]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd3;
              in_raddr <= linaddr(xi_base[3], yi_base_row + 16'd1, i_in_w);
            end
            2'd3: begin
              if (lane_valid[3]) begin
                I01[3]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              in_raddr <= '0;
            end
          endcase
        end

        // ---------------------------------------------------------
        // GATHER I11
        // ---------------------------------------------------------
        S_G11_PRE: begin
          lane_idx <= 2'd0;
          in_raddr <= linaddr(xi_base[0] + 16'd1, yi_base_row + 16'd1, i_in_w);
        end

        S_G11: begin
          case (lane_idx)
            2'd0: begin
              if (lane_valid[0]) begin
                I11[0]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd1;
              in_raddr <= linaddr(xi_base[1] + 16'd1, yi_base_row + 16'd1, i_in_w);
            end
            2'd1: begin
              if (lane_valid[1]) begin
                I11[1]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd2;
              in_raddr <= linaddr(xi_base[2] + 16'd1, yi_base_row + 16'd1, i_in_w);
            end
            2'd2: begin
              if (lane_valid[2]) begin
                I11[2]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              lane_idx <= 2'd3;
              in_raddr <= linaddr(xi_base[3] + 16'd1, yi_base_row + 16'd1, i_in_w);
            end
            2'd3: begin
              if (lane_valid[3]) begin
                I11[3]         <= in_rdata;
                o_mem_rd_count <= o_mem_rd_count + 1;
              end
              in_raddr <= '0;
            end
          endcase
        end

        // ---------------------------------------------------------
        // MUL
        // ---------------------------------------------------------
        S_MUL: begin
          for (i = 0; i < LANES; i = i + 1) begin
            p00_r[i] <= w00_q016[i] * I00[i];
            p10_r[i] <= w10_q016[i] * I10[i];
            p01_r[i] <= w01_q016[i] * I01[i];
            p11_r[i] <= w11_q016[i] * I11[i];
          end
          o_flop_count <= o_flop_count + flops_incr;
        end

        // ---------------------------------------------------------
        // Escrituras
        // ---------------------------------------------------------
        S_WRITE0: begin
          out_waddr <= linaddr(lane_x[0], oy_cur, out_w_reg);
          out_wdata <= PIX_lane[0];
          if (lane_valid[0]) begin
            out_we         <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 1;
          end
        end

        S_WRITE1: begin
          out_waddr <= linaddr(lane_x[1], oy_cur, out_w_reg);
          out_wdata <= PIX_lane[1];
          if (lane_valid[1]) begin
            out_we         <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 1;
          end
        end

        S_WRITE2: begin
          out_waddr <= linaddr(lane_x[2], oy_cur, out_w_reg);
          out_wdata <= PIX_lane[2];
          if (lane_valid[2]) begin
            out_we         <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 1;
          end
        end

        S_WRITE3: begin
          out_waddr <= linaddr(lane_x[3], oy_cur, out_w_reg);
          out_wdata <= PIX_lane[3];
          if (lane_valid[3]) begin
            out_we         <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 1;
          end
        end

        // ---------------------------------------------------------
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
