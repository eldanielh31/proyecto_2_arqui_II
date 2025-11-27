// ============================================================================
// mem_read_controller.sv
//   - Memory read controller for bilinear interpolation
//   - Manages pixel fetching from wide memory (2 read ports)
//   - Extracts 4 neighbor pixels needed for interpolation
//   - Handshake protocol with arithmetic unit
// ============================================================================
module mem_read_controller #(
  parameter int ADDR_W     = 10,
  parameter int IMG_WIDTH  = 64,
  parameter int IMG_HEIGHT = 64
)(
  input  logic         clk,
  input  logic         rst_n,
  
  // Request interface
  input  logic         req_valid,
  input  logic [15:0]  req_xi_base,    // Integer X coordinate (base)
  input  logic [15:0]  req_yi_base,    // Integer Y coordinate (base)
  input  logic [7:0]   req_fx_q,       // Fractional X (Q0.8)
  input  logic [7:0]   req_fy_q,       // Fractional Y (Q0.8)
  output logic         req_ready,
  
  // Data output to arithmetic unit
  output logic         data_valid,
  output logic [7:0]   pixel_tl,       // Top-left I00
  output logic [7:0]   pixel_tr,       // Top-right I10
  output logic [7:0]   pixel_bl,       // Bottom-left I01
  output logic [7:0]   pixel_br,       // Bottom-right I11
  output logic [7:0]   frac_x,
  output logic [7:0]   frac_y,
  input  logic         data_consumed,  // Arithmetic unit consumed data
  
  // Memory interface (2 read ports)
  output logic [ADDR_W-1:0] mem_raddr0,
  input  logic [31:0]       mem_rdata0,
  output logic [ADDR_W-1:0] mem_raddr1,
  input  logic [31:0]       mem_rdata1
);

  // FSM states
  typedef enum logic [2:0] {
    IDLE,
    READ_REQ,
    READ_WAIT,
    EXTRACT,
    DATA_READY
  } state_t;
  
  state_t state, next_state;
  
  // Registered inputs
  logic [15:0] xi_base_r, yi_base_r;
  logic [7:0]  fx_q_r, fy_q_r;
  
  // Memory read data capture
  logic [31:0] row0_data, row1_data;
  
  // Pixel position within 32-bit word
  logic [1:0] pixel_offset;
  
  // Word addresses (divide by 4)
  logic [ADDR_W-1:0] word_addr_row0, word_addr_row1;
  
  // Additional read needed for boundary case
  logic need_extra_read;
  logic [31:0] row0_next_data, row1_next_data;
  logic extra_read_pending;
  
  // Calculate word address and pixel offset
  assign pixel_offset = xi_base_r[1:0];
  
  // --------------------------------------------------------------------------
  // FSM: State register
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= IDLE;
    else
      state <= next_state;
  end
  
  // --------------------------------------------------------------------------
  // FSM: Next state logic
  // --------------------------------------------------------------------------
  always_comb begin
    next_state = state;
    
    case (state)
      IDLE: begin
        if (req_valid)
          next_state = READ_REQ;
      end
      
      READ_REQ: begin
        next_state = READ_WAIT;
      end
      
      READ_WAIT: begin
        if (need_extra_read && !extra_read_pending)
          next_state = READ_WAIT; // Stay for extra read
        else
          next_state = EXTRACT;
      end
      
      EXTRACT: begin
        next_state = DATA_READY;
      end
      
      DATA_READY: begin
        if (data_consumed)
          next_state = IDLE;
      end
      
      default: next_state = IDLE;
    endcase
  end
  
  // --------------------------------------------------------------------------
  // FSM: Datapath
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      req_ready         <= 1'b1;
      data_valid        <= 1'b0;
      mem_raddr0        <= '0;
      mem_raddr1        <= '0;
      xi_base_r         <= '0;
      yi_base_r         <= '0;
      fx_q_r            <= '0;
      fy_q_r            <= '0;
      row0_data         <= '0;
      row1_data         <= '0;
      row0_next_data    <= '0;
      row1_next_data    <= '0;
      need_extra_read   <= 1'b0;
      extra_read_pending<= 1'b0;
      pixel_tl          <= '0;
      pixel_tr          <= '0;
      pixel_bl          <= '0;
      pixel_br          <= '0;
      frac_x            <= '0;
      frac_y            <= '0;
    end else begin
      case (state)
        IDLE: begin
          req_ready  <= 1'b1;
          data_valid <= 1'b0;
          
          if (req_valid) begin
            xi_base_r <= req_xi_base;
            yi_base_r <= req_yi_base;
            fx_q_r    <= req_fx_q;
            fy_q_r    <= req_fy_q;
            
            // Check if we need pixels from adjacent word
            need_extra_read   <= (req_xi_base[1:0] == 2'b11);
            extra_read_pending<= 1'b0;
          end
        end
        
        READ_REQ: begin
          req_ready <= 1'b0;
          
          // Calculate word addresses
          word_addr_row0 = (yi_base_r * (IMG_WIDTH >> 2)) + (xi_base_r >> 2);
          word_addr_row1 = ((yi_base_r + 16'd1) * (IMG_WIDTH >> 2)) + (xi_base_r >> 2);
          
          // Issue primary reads
          mem_raddr0 <= word_addr_row0;
          mem_raddr1 <= word_addr_row1;
        end
        
        READ_WAIT: begin
          if (!extra_read_pending) begin
            // Capture primary read data
            row0_data <= mem_rdata0;
            row1_data <= mem_rdata1;
            
            // If boundary case, issue extra reads
            if (need_extra_read) begin
              mem_raddr0         <= word_addr_row0 + 1;
              mem_raddr1         <= word_addr_row1 + 1;
              extra_read_pending <= 1'b1;
            end
          end else begin
            // Capture extra read data
            row0_next_data     <= mem_rdata0;
            row1_next_data     <= mem_rdata1;
            extra_read_pending <= 1'b0;
          end
        end
        
        EXTRACT: begin
          // Extract 4 pixels based on offset
          // 32-bit word format: {pixel3[31:24], pixel2[23:16], pixel1[15:8], pixel0[7:0]}
          case (pixel_offset)
            2'b00: begin
              pixel_tl <= row0_data[7:0];    // pixel0
              pixel_tr <= row0_data[15:8];   // pixel1
              pixel_bl <= row1_data[7:0];
              pixel_br <= row1_data[15:8];
            end
            2'b01: begin
              pixel_tl <= row0_data[15:8];   // pixel1
              pixel_tr <= row0_data[23:16];  // pixel2
              pixel_bl <= row1_data[15:8];
              pixel_br <= row1_data[23:16];
            end
            2'b10: begin
              pixel_tl <= row0_data[23:16];  // pixel2
              pixel_tr <= row0_data[31:24];  // pixel3
              pixel_bl <= row1_data[23:16];
              pixel_br <= row1_data[31:24];
            end
            2'b11: begin
              pixel_tl <= row0_data[31:24];      // pixel3
              pixel_tr <= row0_next_data[7:0];   // pixel0 from next word
              pixel_bl <= row1_data[31:24];
              pixel_br <= row1_next_data[7:0];
            end
          endcase
          
          frac_x <= fx_q_r;
          frac_y <= fy_q_r;
        end
        
        DATA_READY: begin
          data_valid <= 1'b1;
          if (data_consumed) begin
            data_valid <= 1'b0;
          end
        end
      endcase
    end
  end

endmodule
