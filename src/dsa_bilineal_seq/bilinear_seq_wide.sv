// ============================================================================
// bilinear_seq_wide.sv
//   - Sequential bilinear interpolation adapted for wide memory
//   - Uses mem_read_controller and bilinear_arith modules
//   - Compatible with 32-bit wide memory (2 read ports)
// ============================================================================
module bilinear_seq_wide #(
  parameter int AW = 10  // Reduced by 2 bits for wide memory
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

  // Wide memory read interface (2 ports)
  output logic [AW-1:0] in_raddr0,
  input  logic [31:0]   in_rdata0,
  output logic [AW-1:0] in_raddr1,
  input  logic [31:0]   in_rdata1,

  // Memory write interface
  output logic [AW-1:0] out_waddr,
  output logic [7:0]    out_wdata,
  output logic          out_we,

  // Performance counters
  output logic [31:0]  o_flop_count,
  output logic [31:0]  o_mem_rd_count,
  output logic [31:0]  o_mem_wr_count
);

  localparam logic [31:0] FLOPS_PER_PIXEL = 32'd11;

  // FSM
  typedef enum logic [3:0] {
    S_IDLE,
    S_INIT,
    S_ROW_INIT,
    S_PIXEL_START,
    S_MEM_WAIT,
    S_ARITH_WAIT,
    S_WRITE,
    S_STEP_WAIT,
    S_ADVANCE,
    S_DONE
  } state_t;
  
  state_t state, next_state;

  // Dimension calculation
  logic [15:0] out_w_reg, out_h_reg;
  logic [31:0] mul_w, mul_h;
  assign mul_w = i_in_w * i_scale_q88;
  assign mul_h = i_in_h * i_scale_q88;

  logic [15:0] inv_scale_q88;
  
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
  logic [15:0] ox_cur, oy_cur;

  // Source coordinates Q16.8
  logic [31:0] sx_fix32, sy_fix32;
  assign sx_fix32 = ox_cur * inv_scale_q88;
  assign sy_fix32 = oy_cur * inv_scale_q88;

  logic [15:0] sx_int, sy_int;
  logic [7:0]  ax_q, ay_q;
  assign sx_int = sx_fix32[23:8];
  assign sy_int = sy_fix32[23:8];
  assign ax_q   = sx_fix32[7:0];
  assign ay_q   = sy_fix32[7:0];

  // Clamped coordinates
  logic [15:0] xi_base, yi_base;
  logic [7:0]  fx_q, fy_q;
  logic [15:0] xi_base_next, yi_base_next;
  logic [7:0]  fx_q_next, fy_q_next;

  // Memory controller interface
  logic        mem_req_valid;
  logic        mem_req_ready;
  logic        mem_data_valid;
  logic [7:0]  mem_pixel_tl, mem_pixel_tr, mem_pixel_bl, mem_pixel_br;
  logic [7:0]  mem_frac_x, mem_frac_y;
  logic        mem_data_consumed;

  // Arithmetic unit interface
  logic        arith_data_valid;
  logic        arith_data_consumed;
  logic        arith_result_valid;
  logic [7:0]  arith_result_pixel;
  logic        arith_result_consumed;

  // Instantiate memory read controller
  mem_read_controller #(
    .ADDR_W(AW),
    .IMG_WIDTH(64),  // Match your image size
    .IMG_HEIGHT(64)
  ) u_mem_ctrl (
    .clk(clk),
    .rst_n(rst_n),
    .req_valid(mem_req_valid),
    .req_xi_base(xi_base),
    .req_yi_base(yi_base),
    .req_fx_q(fx_q),
    .req_fy_q(fy_q),
    .req_ready(mem_req_ready),
    .data_valid(mem_data_valid),
    .pixel_tl(mem_pixel_tl),
    .pixel_tr(mem_pixel_tr),
    .pixel_bl(mem_pixel_bl),
    .pixel_br(mem_pixel_br),
    .frac_x(mem_frac_x),
    .frac_y(mem_frac_y),
    .data_consumed(mem_data_consumed),
    .mem_raddr0(in_raddr0),
    .mem_rdata0(in_rdata0),
    .mem_raddr1(in_raddr1),
    .mem_rdata1(in_rdata1)
  );

  // Instantiate arithmetic unit
  bilinear_arith u_arith (
    .clk(clk),
    .rst_n(rst_n),
    .data_valid(arith_data_valid),
    .pixel_tl(mem_pixel_tl),
    .pixel_tr(mem_pixel_tr),
    .pixel_bl(mem_pixel_bl),
    .pixel_br(mem_pixel_br),
    .frac_x(mem_frac_x),
    .frac_y(mem_frac_y),
    .data_consumed(arith_data_consumed),
    .result_valid(arith_result_valid),
    .result_pixel(arith_result_pixel),
    .result_consumed(arith_result_consumed)
  );

  // Connect memory data to arithmetic unit
  assign arith_data_valid = mem_data_valid;
  assign mem_data_consumed = arith_data_consumed;

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

  // Main FSM
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= S_IDLE;
      busy  <= 1'b0;
      done  <= 1'b0;
      out_we <= 1'b0;
      
      out_waddr <= '0;
      out_wdata <= 8'h00;
      
      ox_cur <= 16'd0;
      oy_cur <= 16'd0;
      
      out_w_reg <= 16'd0;
      out_h_reg <= 16'd0;
      inv_scale_q88 <= 16'd0;
      
      o_out_w <= 16'd0;
      o_out_h <= 16'd0;
      
      xi_base <= 16'd0;
      yi_base <= 16'd0;
      fx_q <= 8'd0;
      fy_q <= 8'd0;
      
      xi_base_next <= 16'd0;
      yi_base_next <= 16'd0;
      fx_q_next <= 8'd0;
      fy_q_next <= 8'd0;
      
      mem_req_valid <= 1'b0;
      arith_result_consumed <= 1'b0;
      
      o_flop_count <= 32'd0;
      o_mem_rd_count <= 32'd0;
      o_mem_wr_count <= 32'd0;
      
    end else begin
      state <= next_state;
      out_we <= 1'b0;
      mem_req_valid <= 1'b0;
      arith_result_consumed <= 1'b0;
      
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
            o_flop_count <= 32'd0;
            o_mem_rd_count <= 32'd0;
            o_mem_wr_count <= 32'd0;
          end
        end
        
        S_INIT: begin
          busy <= 1'b1;
          ox_cur <= 16'd0;
          oy_cur <= 16'd0;
        end
        
        S_ROW_INIT: begin
          ox_cur <= 16'd0;
        end
        
        S_PIXEL_START: begin
          // Compute and clamp coordinates
          xi_base_next = sx_int;
          yi_base_next = sy_int;
          fx_q_next = ax_q;
          fy_q_next = ay_q;
          
          if (sx_int >= i_in_w - 16'd1) begin
            xi_base_next = i_in_w - 16'd2;
            fx_q_next = 8'hFF;
          end
          
          if (sy_int >= i_in_h - 16'd1) begin
            yi_base_next = i_in_h - 16'd2;
            fy_q_next = 8'hFF;
          end
          
          xi_base <= xi_base_next;
          yi_base <= yi_base_next;
          fx_q <= fx_q_next;
          fy_q <= fy_q_next;
          
          mem_req_valid <= 1'b1;
        end
        
        S_MEM_WAIT: begin
          // Wait for memory controller to provide data
          if (mem_data_valid) begin
            o_mem_rd_count <= o_mem_rd_count + 32'd4; // 4 neighbor reads
          end
        end
        
        S_ARITH_WAIT: begin
          // Wait for arithmetic unit to complete
          if (arith_result_valid) begin
            o_flop_count <= o_flop_count + FLOPS_PER_PIXEL;
          end
        end
        
        S_WRITE: begin
          if (arith_result_valid) begin
            out_waddr <= linaddr(ox_cur, oy_cur, out_w_reg);
            out_wdata <= arith_result_pixel;
            out_we <= 1'b1;
            arith_result_consumed <= 1'b1;
            o_mem_wr_count <= o_mem_wr_count + 32'd1;
          end
        end
        
        S_STEP_WAIT: begin
          // Debug stepping mode
        end
        
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
      S_ROW_INIT: next_state = S_PIXEL_START;
      S_PIXEL_START: next_state = S_MEM_WAIT;
      S_MEM_WAIT: if (mem_data_valid) next_state = S_ARITH_WAIT;
      S_ARITH_WAIT: if (arith_result_valid) next_state = S_WRITE;
      
      S_WRITE: begin
        if (i_step_en)
          next_state = S_STEP_WAIT;
        else if ((ox_cur + 16'd1 >= out_w_reg) && (oy_cur + 16'd1 >= out_h_reg))
          next_state = S_DONE;
        else
          next_state = S_ADVANCE;
      end
      
      S_STEP_WAIT: begin
        if (i_step_pulse) begin
          if ((ox_cur + 16'd1 >= out_w_reg) && (oy_cur + 16'd1 >= out_h_reg))
            next_state = S_DONE;
          else
            next_state = S_ADVANCE;
        end
      end
      
      S_ADVANCE: begin
        if ((ox_cur + 16'd1 >= out_w_reg) && (oy_cur + 16'd1 >= out_h_reg))
          next_state = S_DONE;
        else if (ox_cur + 16'd1 >= out_w_reg)
          next_state = S_ROW_INIT;
        else
          next_state = S_PIXEL_START;
      end
      
      S_DONE: next_state = S_IDLE;
      default: next_state = S_IDLE;
    endcase
  end

endmodule
