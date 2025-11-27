// ============================================================================
// bilinear_simd4_wide.sv
//   - SIMD x4 bilinear interpolation adapted for wide memory
//   - Processes 4 pixels per cycle using shared wide memory
//   - Uses time-multiplexed access to 2 read ports
// ============================================================================
module bilinear_simd4_wide #(
  parameter int AW = 10,
  parameter int IMG_W  = 16,
  parameter int IMG_H  = 16
)(
  input  logic        clk,
  input  logic        rst_n,

  // Control
  input  logic        start,
  output logic        busy,
  output logic        done,

  // Stepping
  input  logic        i_step_en,
  input  logic        i_step_pulse,

  // Image parameters
  input  logic [15:0] i_in_w,
  input  logic [15:0] i_in_h,
  input  logic [15:0] i_scale_q88,

  output logic [15:0] o_out_w,
  output logic [15:0] o_out_h,

  // Wide memory read interface (2 ports, shared across lanes)
  output logic [AW-1:0] in_raddr0,
  input  logic [31:0]   in_rdata0,
  output logic [AW-1:0] in_raddr1,
  input  logic [31:0]   in_rdata1,

  // Memory write interface (4 ports for SIMD)
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

  // Performance counters
  output logic [31:0]  o_flop_count,
  output logic [31:0]  o_mem_rd_count,
  output logic [31:0]  o_mem_wr_count
);

  localparam logic [31:0] FLOPS_PER_PIXEL = 32'd11;
  localparam logic [8:0]  ONE_Q08 = 9'd256;
  localparam logic [31:0] ROUND_Q016 = 32'h0000_8000;

  // FSM
  typedef enum logic [4:0] {
    S_IDLE,
    S_INIT,
    S_ROW_INIT,
    S_GROUP_SETUP,
    S_LANE0_REQ,
    S_LANE0_WAIT,
    S_LANE1_REQ,
    S_LANE1_WAIT,
    S_LANE2_REQ,
    S_LANE2_WAIT,
    S_LANE3_REQ,
    S_LANE3_WAIT,
    S_ARITH_ALL,
    S_WRITE,
    S_STEP_WAIT,
    S_ADVANCE,
    S_DONE
  } state_t;
  
  state_t state, next_state;

  // Dimensions
  logic [15:0] out_w_reg, out_h_reg;
  logic [31:0] mul_w, mul_h;
  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  logic [15:0] inv_scale_q88;
  logic [15:0] groups_per_row;
  
  function automatic logic [15:0] inv_q88(input logic [15:0] scale_q88);
    logic [31:0] num;
  begin
    if (scale_q88 == 16'd0)
      inv_q88 = 16'hFFFF;
    else begin
      num = 32'd65536;
      inv_q88 = (num / scale_q88);
    end
  end
  endfunction

  // Output coordinates
  logic [15:0] oy_cur, group_x;

  // Q16.8 coordinates
  logic [23:0] sy_fix_row, sx_fix_group;
  logic [23:0] inv_step_q168, four_step_q168;
  assign inv_step_q168 = {8'd0, inv_scale_q88};
  assign four_step_q168 = inv_step_q168 << 2;

  // Per-lane data
  logic [15:0] ox_lane[0:3];
  logic [15:0] xi_base_lane[0:3];
  logic [15:0] yi_base_row;
  logic [7:0]  fx_q_lane[0:3];
  logic [7:0]  fy_q_row;

  // Per-lane pixels
  logic [7:0] I00[0:3], I10[0:3], I01[0:3], I11[0:3];

  // Per-lane arithmetic (weights, products, results)
  logic [8:0]  wx0_lane[0:3], wx1_lane[0:3];
  logic [8:0]  wy0_row, wy1_row;
  logic [17:0] w00_lane[0:3], w10_lane[0:3], w01_lane[0:3], w11_lane[0:3];
  logic [31:0] p00_r[0:3], p10_r[0:3], p01_r[0:3], p11_r[0:3];
  logic [31:0] sum_lane[0:3], sum_rounded_lane[0:3];
  logic [7:0]  pix_lane[0:3];

  // Memory controller interface per lane
  logic [3:0]  lane_req_valid;
  logic [3:0]  lane_req_ready;
  logic [3:0]  lane_data_valid;
  logic [3:0]  lane_data_consumed;
  
  logic [7:0]  lane_pixel_tl[0:3], lane_pixel_tr[0:3];
  logic [7:0]  lane_pixel_bl[0:3], lane_pixel_br[0:3];

  // Current lane being processed
  logic [1:0] current_lane;

  // Memory controller instance (shared, multiplexed)
  logic        mem_ctrl_req_valid;
  logic [15:0] mem_ctrl_xi_base;
  logic [15:0] mem_ctrl_yi_base;
  logic [7:0]  mem_ctrl_fx_q;
  logic [7:0]  mem_ctrl_fy_q;
  logic        mem_ctrl_req_ready;
  logic        mem_ctrl_data_valid;
  logic [7:0]  mem_ctrl_pixel_tl, mem_ctrl_pixel_tr;
  logic [7:0]  mem_ctrl_pixel_bl, mem_ctrl_pixel_br;
  logic        mem_ctrl_data_consumed;

  mem_read_controller #(
    .ADDR_W(AW),
    .IMG_WIDTH (IMG_W),   // antes 16
    .IMG_HEIGHT(IMG_H)    // antes 16
  ) u_mem_ctrl (
    .clk(clk),
    .rst_n(rst_n),
    .req_valid(mem_ctrl_req_valid),
    .req_xi_base(mem_ctrl_xi_base),
    .req_yi_base(mem_ctrl_yi_base),
    .req_fx_q(mem_ctrl_fx_q),
    .req_fy_q(mem_ctrl_fy_q),
    .req_ready(mem_ctrl_req_ready),
    .data_valid(mem_ctrl_data_valid),
    .pixel_tl(mem_ctrl_pixel_tl),
    .pixel_tr(mem_ctrl_pixel_tr),
    .pixel_bl(mem_ctrl_pixel_bl),
    .pixel_br(mem_ctrl_pixel_br),
    .frac_x(),
    .frac_y(),
    .data_consumed(mem_ctrl_data_consumed),
    .mem_raddr0(in_raddr0),
    .mem_rdata0(in_rdata0),
    .mem_raddr1(in_raddr1),
    .mem_rdata1(in_rdata1)
  );

  // Weights calculation
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
      
      assign sum_lane[gv] = p00_r[gv] + p10_r[gv] + p01_r[gv] + p11_r[gv];
      assign sum_rounded_lane[gv] = sum_lane[gv] + ROUND_Q016;
      assign pix_lane[gv] = sum_rounded_lane[gv][23:16];
    end
  endgenerate

  // Address calculation
  function automatic logic [AW-1:0] linaddr(
    input logic [15:0] x,
    input logic [15:0] y,
    input logic [15:0] width
  );
    logic [31:0] tmp;
  begin
    tmp = (y * width) + x;
    linaddr = tmp[AW-1:0];
  end
  endfunction

  integer i, n_wr, n_pix;
  logic [23:0] sx_loc;
  logic [15:0] sy_int_row;
  logic [7:0]  ay_q_row;
  logic [15:0] yi_tmp, xi_tmp_lane[0:3];
  logic [7:0]  fy_tmp, fx_tmp_lane[0:3];

  assign sy_int_row = sy_fix_row[23:8];
  assign ay_q_row = sy_fix_row[7:0];

  // Main FSM
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= S_IDLE;
      busy <= 1'b0;
      done <= 1'b0;
      
      out_w_reg <= 16'd0;
      out_h_reg <= 16'd0;
      o_out_w <= 16'd0;
      o_out_h <= 16'd0;
      
      inv_scale_q88 <= 16'd0;
      groups_per_row <= 16'd0;
      
      oy_cur <= 16'd0;
      group_x <= 16'd0;
      
      sy_fix_row <= 24'd0;
      sx_fix_group <= 24'd0;
      
      yi_base_row <= 16'd0;
      fy_q_row <= 8'd0;
      
      current_lane <= 2'd0;
      mem_ctrl_req_valid <= 1'b0;
      mem_ctrl_data_consumed <= 1'b0;
      
      for (i = 0; i < 4; i = i + 1) begin
        ox_lane[i] <= 16'd0;
        xi_base_lane[i] <= 16'd0;
        fx_q_lane[i] <= 8'd0;
        I00[i] <= 8'd0; I10[i] <= 8'd0;
        I01[i] <= 8'd0; I11[i] <= 8'd0;
        p00_r[i] <= 32'd0; p10_r[i] <= 32'd0;
        p01_r[i] <= 32'd0; p11_r[i] <= 32'd0;
      end
      
      out_we0 <= 1'b0; out_we1 <= 1'b0;
      out_we2 <= 1'b0; out_we3 <= 1'b0;
      
      o_flop_count <= 32'd0;
      o_mem_rd_count <= 32'd0;
      o_mem_wr_count <= 32'd0;
      
    end else begin
      state <= next_state;
      
      out_we0 <= 1'b0; out_we1 <= 1'b0;
      out_we2 <= 1'b0; out_we3 <= 1'b0;
      mem_ctrl_req_valid <= 1'b0;
      mem_ctrl_data_consumed <= 1'b0;
      
      case (state)
        S_IDLE: begin
          done <= 1'b0;
          busy <= 1'b0;
          if (start) begin
            out_w_reg <= mul_w[23:8];
            out_h_reg <= mul_h[23:8];
            o_out_w <= mul_w[23:8];
            o_out_h <= mul_h[23:8];
            inv_scale_q88 <= inv_q88(i_scale_q88);
            groups_per_row <= (mul_w[23:8] + 16'd3) >> 2;
            o_flop_count <= 32'd0;
            o_mem_rd_count <= 32'd0;
            o_mem_wr_count <= 32'd0;
          end
        end
        
        S_INIT: begin
          busy <= 1'b1;
          oy_cur <= 16'd0;
          group_x <= 16'd0;
          sy_fix_row <= 24'd0;
          sx_fix_group <= 24'd0;
        end
        
        S_ROW_INIT: begin
          group_x <= 16'd0;
          sx_fix_group <= 24'd0;
        end
        
        S_GROUP_SETUP: begin
          // Y clamping
          yi_tmp = sy_int_row;
          fy_tmp = ay_q_row;
          if (sy_int_row >= i_in_h - 16'd1) begin
            yi_tmp = i_in_h - 16'd2;
            fy_tmp = 8'hFF;
          end
          yi_base_row <= yi_tmp;
          fy_q_row <= fy_tmp;
          
          // X per lane
          for (i = 0; i < 4; i = i + 1) begin
            ox_lane[i] <= (group_x << 2) + i[15:0];
            
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
            
            xi_base_lane[i] <= xi_tmp_lane[i];
            fx_q_lane[i] <= fx_tmp_lane[i];
          end
          
          current_lane <= 2'd0;
        end
        
        S_LANE0_REQ, S_LANE1_REQ, S_LANE2_REQ, S_LANE3_REQ: begin
          mem_ctrl_req_valid <= 1'b1;
          mem_ctrl_xi_base <= xi_base_lane[current_lane];
          mem_ctrl_yi_base <= yi_base_row;
          mem_ctrl_fx_q <= fx_q_lane[current_lane];
          mem_ctrl_fy_q <= fy_q_row;
        end
        
        S_LANE0_WAIT, S_LANE1_WAIT, S_LANE2_WAIT, S_LANE3_WAIT: begin
          if (mem_ctrl_data_valid) begin
            I00[current_lane] <= mem_ctrl_pixel_tl;
            I10[current_lane] <= mem_ctrl_pixel_tr;
            I01[current_lane] <= mem_ctrl_pixel_bl;
            I11[current_lane] <= mem_ctrl_pixel_br;
            mem_ctrl_data_consumed <= 1'b1;
            o_mem_rd_count <= o_mem_rd_count + 32'd4;
            current_lane <= current_lane + 2'd1;
          end
        end
        
        S_ARITH_ALL: begin
          for (i = 0; i < 4; i = i + 1) begin
            p00_r[i] <= w00_lane[i] * I00[i];
            p10_r[i] <= w10_lane[i] * I10[i];
            p01_r[i] <= w01_lane[i] * I01[i];
            p11_r[i] <= w11_lane[i] * I11[i];
          end
        end
        
        S_WRITE: begin
          n_wr = 0;
          n_pix = 0;
          
          if (ox_lane[0] < out_w_reg) begin
            out_waddr0 <= linaddr(ox_lane[0], oy_cur, out_w_reg);
            out_wdata0 <= pix_lane[0];
            out_we0 <= 1'b1;
            n_wr = n_wr + 1;
            n_pix = n_pix + 1;
          end
          
          if (ox_lane[1] < out_w_reg) begin
            out_waddr1 <= linaddr(ox_lane[1], oy_cur, out_w_reg);
            out_wdata1 <= pix_lane[1];
            out_we1 <= 1'b1;
            n_wr = n_wr + 1;
            n_pix = n_pix + 1;
          end
          
          if (ox_lane[2] < out_w_reg) begin
            out_waddr2 <= linaddr(ox_lane[2], oy_cur, out_w_reg);
            out_wdata2 <= pix_lane[2];
            out_we2 <= 1'b1;
            n_wr = n_wr + 1;
            n_pix = n_pix + 1;
          end
          
          if (ox_lane[3] < out_w_reg) begin
            out_waddr3 <= linaddr(ox_lane[3], oy_cur, out_w_reg);
            out_wdata3 <= pix_lane[3];
            out_we3 <= 1'b1;
            n_wr = n_wr + 1;
            n_pix = n_pix + 1;
          end
          
          o_mem_wr_count <= o_mem_wr_count + n_wr;
          o_flop_count <= o_flop_count + (n_pix * FLOPS_PER_PIXEL);
        end
        
        S_ADVANCE: begin
          if (group_x + 16'd1 < groups_per_row) begin
            group_x <= group_x + 16'd1;
            sx_fix_group <= sx_fix_group + four_step_q168;
          end else begin
            if (oy_cur + 16'd1 < out_h_reg) begin
              oy_cur <= oy_cur + 16'd1;
              sy_fix_row <= sy_fix_row + inv_step_q168;
            end
          end
        end
        
        S_DONE: begin
          busy <= 1'b0;
          done <= 1'b1;
        end
      endcase
    end
  end

  // Next state logic
  always_comb begin
    next_state = state;
    
    case (state)
      S_IDLE: if (start) next_state = S_INIT;
      S_INIT: next_state = S_ROW_INIT;
      S_ROW_INIT: next_state = S_GROUP_SETUP;
      S_GROUP_SETUP: next_state = S_LANE0_REQ;
      
      S_LANE0_REQ: next_state = S_LANE0_WAIT;
      S_LANE0_WAIT: if (mem_ctrl_data_valid) next_state = S_LANE1_REQ;
      
      S_LANE1_REQ: next_state = S_LANE1_WAIT;
      S_LANE1_WAIT: if (mem_ctrl_data_valid) next_state = S_LANE2_REQ;
      
      S_LANE2_REQ: next_state = S_LANE2_WAIT;
      S_LANE2_WAIT: if (mem_ctrl_data_valid) next_state = S_LANE3_REQ;
      
      S_LANE3_REQ: next_state = S_LANE3_WAIT;
      S_LANE3_WAIT: if (mem_ctrl_data_valid) next_state = S_ARITH_ALL;
      
      S_ARITH_ALL: next_state = S_WRITE;
      
      S_WRITE: begin
        if (i_step_en)
          next_state = S_STEP_WAIT;
        else if ((group_x + 16'd1 >= groups_per_row) && (oy_cur + 16'd1 >= out_h_reg))
          next_state = S_DONE;
        else
          next_state = S_ADVANCE;
      end
      
      S_STEP_WAIT: begin
        if (i_step_pulse) begin
          if ((group_x + 16'd1 >= groups_per_row) && (oy_cur + 16'd1 >= out_h_reg))
            next_state = S_DONE;
          else
            next_state = S_ADVANCE;
        end
      end
      
      S_ADVANCE: begin
        if ((group_x + 16'd1 >= groups_per_row) && (oy_cur + 16'd1 >= out_h_reg))
          next_state = S_DONE;
        else if (group_x + 16'd1 >= groups_per_row)
          next_state = S_ROW_INIT;
        else
          next_state = S_GROUP_SETUP;
      end
      
      S_DONE: next_state = S_IDLE;
      default: next_state = S_IDLE;
    endcase
  end

endmodule