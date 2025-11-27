// ============================================================================
// bilinear_arith.sv
//   - Bilinear interpolation arithmetic unit
//   - Takes 4 neighbor pixels and fractional coordinates
//   - Computes interpolated pixel value using Q8.8 arithmetic
// ============================================================================
module bilinear_arith (
  input  logic        clk,
  input  logic        rst_n,
  
  // Input data
  input  logic        data_valid,
  input  logic [7:0]  pixel_tl,    // I00
  input  logic [7:0]  pixel_tr,    // I10
  input  logic [7:0]  pixel_bl,    // I01
  input  logic [7:0]  pixel_br,    // I11
  input  logic [7:0]  frac_x,      // fx_q (Q0.8)
  input  logic [7:0]  frac_y,      // fy_q (Q0.8)
  output logic        data_consumed,
  
  // Output result
  output logic        result_valid,
  output logic [7:0]  result_pixel,
  input  logic        result_consumed
);

  localparam logic [8:0]  ONE_Q08    = 9'd256;
  localparam logic [31:0] ROUND_Q016 = 32'h0000_8000;
  
  // Pipeline stages
  typedef enum logic [2:0] {
    IDLE,
    CALC_WEIGHTS,
    MULTIPLY,
    SUM,
    DONE
  } state_t;
  
  state_t state, next_state;
  
  // Registered inputs
  logic [7:0] I00_r, I10_r, I01_r, I11_r;
  logic [7:0] fx_r, fy_r;
  
  // Weights (Q0.8 -> Q0.16)
  logic [8:0]  wx0, wx1, wy0, wy1;
  logic [17:0] w00, w10, w01, w11;
  
  // Products (Q8.16)
  logic [31:0] p00, p10, p01, p11;
  
  // Sum and result
  logic [31:0] sum_q016, sum_rounded;
  logic [7:0]  pix_final;
  
  // Combinational weight calculation
  assign wx0 = ONE_Q08 - {1'b0, fx_r};
  assign wx1 = {1'b0, fx_r};
  assign wy0 = ONE_Q08 - {1'b0, fy_r};
  assign wy1 = {1'b0, fy_r};
  
  assign w00 = wx0 * wy0;
  assign w10 = wx1 * wy0;
  assign w01 = wx0 * wy1;
  assign w11 = wx1 * wy1;
  
  // Sum and round
  assign sum_q016    = p00 + p10 + p01 + p11;
  assign sum_rounded = sum_q016 + ROUND_Q016;
  assign pix_final   = sum_rounded[23:16];
  
  // FSM
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state          <= IDLE;
      data_consumed  <= 1'b0;
      result_valid   <= 1'b0;
      result_pixel   <= 8'h00;
      I00_r <= 8'h00; I10_r <= 8'h00;
      I01_r <= 8'h00; I11_r <= 8'h00;
      fx_r  <= 8'h00; fy_r  <= 8'h00;
      p00   <= 32'h0; p10   <= 32'h0;
      p01   <= 32'h0; p11   <= 32'h0;
    end else begin
      state <= next_state;
      
      data_consumed <= 1'b0;
      
      case (state)
        IDLE: begin
          result_valid <= 1'b0;
          if (data_valid) begin
            I00_r <= pixel_tl;
            I10_r <= pixel_tr;
            I01_r <= pixel_bl;
            I11_r <= pixel_br;
            fx_r  <= frac_x;
            fy_r  <= frac_y;
            data_consumed <= 1'b1;
          end
        end
        
        CALC_WEIGHTS: begin
          // Weights calculated combinationally
        end
        
        MULTIPLY: begin
          p00 <= w00 * I00_r;
          p10 <= w10 * I10_r;
          p01 <= w01 * I01_r;
          p11 <= w11 * I11_r;
        end
        
        SUM: begin
          result_pixel <= pix_final;
        end
        
        DONE: begin
          result_valid <= 1'b1;
          if (result_consumed) begin
            result_valid <= 1'b0;
          end
        end
      endcase
    end
  end
  
  // Next state logic
  always_comb begin
    next_state = state;
    case (state)
      IDLE:         if (data_valid) next_state = CALC_WEIGHTS;
      CALC_WEIGHTS: next_state = MULTIPLY;
      MULTIPLY:     next_state = SUM;
      SUM:          next_state = DONE;
      DONE:         if (result_consumed) next_state = IDLE;
      default:      next_state = IDLE;
    endcase
  end

endmodule