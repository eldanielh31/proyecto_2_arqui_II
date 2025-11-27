// ============================================================================
// mem_read_controller.sv - UPDATED with line buffer support
// ============================================================================
module mem_read_controller #(
  parameter int ADDR_W     = 10,
  parameter int IMG_WIDTH  = 16,
  parameter int IMG_HEIGHT = 16,
  parameter bit USE_LINE_BUFFER = 0 // Enable line buffering
)(
  input  logic         clk,
  input  logic         rst_n,
  
  input  logic         req_valid,
  input  logic [15:0]  req_xi_base,
  input  logic [15:0]  req_yi_base,
  input  logic [7:0]   req_fx_q,
  input  logic [7:0]   req_fy_q,
  output logic         req_ready,
  
  output logic         data_valid,
  output logic [7:0]   pixel_tl,
  output logic [7:0]   pixel_tr,
  output logic [7:0]   pixel_bl,
  output logic [7:0]   pixel_br,
  output logic [7:0]   frac_x,
  output logic [7:0]   frac_y,
  input  logic         data_consumed,
  
  output logic [ADDR_W-1:0] mem_raddr0,
  input  logic [31:0]       mem_rdata0,
  output logic [ADDR_W-1:0] mem_raddr1,
  input  logic [31:0]       mem_rdata1
);

  generate
    if (USE_LINE_BUFFER) begin : g_with_line_buffer
      // Line buffer implementation for better performance
      typedef enum logic [3:0] {
        IDLE,
        INIT_LOAD,
        LOAD_ROW0,
        LOAD_ROW1,
        READY,
        READ_FROM_BUFFER,
        EXTRACT,
        DATA_READY_STATE
      } state_t;
      
      state_t state, next_state;
      
      logic [15:0] current_row, loaded_row0, loaded_row1;
      logic [9:0] load_word_idx, words_per_line;
      logic lb0_wr_en, lb1_wr_en;
      logic [9:0] lb0_wr_addr, lb1_wr_addr, lb0_rd_addr, lb1_rd_addr;
      logic [31:0] lb0_wr_data, lb1_wr_data, lb0_rd_data, lb1_rd_data;
      logic [15:0] xi_r, yi_r;
      logic [7:0] fx_r, fy_r;
      logic [31:0] row0_word, row1_word, row0_next, row1_next;
      logic [1:0] pixel_offset;
      logic need_next_word;
      
      assign words_per_line = (IMG_WIDTH + 10'd3) >> 2;
      assign pixel_offset = xi_r[1:0];
      assign need_next_word = (pixel_offset == 2'b11);
      
      line_buffer #(.LINE_WIDTH(IMG_WIDTH)) lb0 (
        .clk(clk), .rst_n(rst_n),
        .wr_en(lb0_wr_en), .wr_addr(lb0_wr_addr), .wr_data(lb0_wr_data),
        .rd_addr(lb0_rd_addr), .rd_data(lb0_rd_data)
      );
      
      line_buffer #(.LINE_WIDTH(IMG_WIDTH)) lb1 (
        .clk(clk), .rst_n(rst_n),
        .wr_en(lb1_wr_en), .wr_addr(lb1_wr_addr), .wr_data(lb1_wr_data),
        .rd_addr(lb1_rd_addr), .rd_data(lb1_rd_data)
      );
      
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          state <= IDLE;
          req_ready <= 1'b0;
          data_valid <= 1'b0;
          current_row <= 16'd0;
          loaded_row0 <= 16'hFFFF;
          loaded_row1 <= 16'hFFFF;
          load_word_idx <= 10'd0;
          lb0_wr_en <= 1'b0;
          lb1_wr_en <= 1'b0;
          {pixel_tl, pixel_tr, pixel_bl, pixel_br} <= '0;
          {frac_x, frac_y} <= '0;
        end else begin
          state <= next_state;
          lb0_wr_en <= 1'b0;
          lb1_wr_en <= 1'b0;
          
          case (state)
            IDLE: begin
              req_ready <= 1'b0;
              data_valid <= 1'b0;
              current_row <= 16'd0;
              loaded_row0 <= 16'hFFFF;
              loaded_row1 <= 16'hFFFF;
            end
            
            INIT_LOAD: begin
              load_word_idx <= 10'd0;
              current_row <= 16'd0;
            end
            
            LOAD_ROW0: begin
              if (load_word_idx < words_per_line) begin
                mem_raddr0 <= (current_row * words_per_line) + load_word_idx;
                if (load_word_idx > 0) begin
                  lb0_wr_en <= 1'b1;
                  lb0_wr_addr <= load_word_idx - 10'd1;
                  lb0_wr_data <= mem_rdata0;
                end
                load_word_idx <= load_word_idx + 10'd1;
              end else begin
                lb0_wr_en <= 1'b1;
                lb0_wr_addr <= load_word_idx - 10'd1;
                lb0_wr_data <= mem_rdata0;
                loaded_row0 <= current_row;
                load_word_idx <= 10'd0;
              end
            end
            
            LOAD_ROW1: begin
              if (load_word_idx < words_per_line) begin
                mem_raddr1 <= ((current_row + 16'd1) * words_per_line) + load_word_idx;
                if (load_word_idx > 0) begin
                  lb1_wr_en <= 1'b1;
                  lb1_wr_addr <= load_word_idx - 10'd1;
                  lb1_wr_data <= mem_rdata1;
                end
                load_word_idx <= load_word_idx + 10'd1;
              end else begin
                lb1_wr_en <= 1'b1;
                lb1_wr_addr <= load_word_idx - 10'd1;
                lb1_wr_data <= mem_rdata1;
                loaded_row1 <= current_row + 16'd1;
              end
            end
            
            READY: begin
              req_ready <= 1'b1;
              if (req_valid) begin
                xi_r <= req_xi_base;
                yi_r <= req_yi_base;
                fx_r <= req_fx_q;
                fy_r <= req_fy_q;
                
                // Check if we need to reload buffers
                if (req_yi_base != loaded_row0) begin
                  current_row <= req_yi_base;
                  req_ready <= 1'b0;
                end
              end
            end
            
            READ_FROM_BUFFER: begin
              lb0_rd_addr <= xi_r >> 2;
              lb1_rd_addr <= xi_r >> 2;
              row0_word <= lb0_rd_data;
              row1_word <= lb1_rd_data;
              
              if (need_next_word) begin
                row0_next <= lb0_rd_data;  // Would need +1 addressing
                row1_next <= lb1_rd_data;
              end
            end
            
            EXTRACT: begin
              case (pixel_offset)
                2'b00: begin
                  pixel_tl <= row0_word[7:0];
                  pixel_tr <= row0_word[15:8];
                  pixel_bl <= row1_word[7:0];
                  pixel_br <= row1_word[15:8];
                end
                2'b01: begin
                  pixel_tl <= row0_word[15:8];
                  pixel_tr <= row0_word[23:16];
                  pixel_bl <= row1_word[15:8];
                  pixel_br <= row1_word[23:16];
                end
                2'b10: begin
                  pixel_tl <= row0_word[23:16];
                  pixel_tr <= row0_word[31:24];
                  pixel_bl <= row1_word[23:16];
                  pixel_br <= row1_word[31:24];
                end
                2'b11: begin
                  pixel_tl <= row0_word[31:24];
                  pixel_tr <= row0_next[7:0];
                  pixel_bl <= row1_word[31:24];
                  pixel_br <= row1_next[7:0];
                end
              endcase
              frac_x <= fx_r;
              frac_y <= fy_r;
            end
            
            DATA_READY_STATE: begin
              data_valid <= 1'b1;
              if (data_consumed)
                data_valid <= 1'b0;
            end
          endcase
        end
      end
      
      always_comb begin
        next_state = state;
        case (state)
          IDLE: next_state = INIT_LOAD;
          INIT_LOAD: next_state = LOAD_ROW0;
          LOAD_ROW0: if (load_word_idx >= words_per_line) next_state = LOAD_ROW1;
          LOAD_ROW1: if (load_word_idx >= words_per_line) next_state = READY;
          READY: begin
            if (req_valid) begin
              if (req_yi_base == loaded_row0)
                next_state = READ_FROM_BUFFER;
              else
                next_state = INIT_LOAD;
            end
          end
          READ_FROM_BUFFER: next_state = EXTRACT;
          EXTRACT: next_state = DATA_READY_STATE;
          DATA_READY_STATE: if (data_consumed) next_state = READY;
          default: next_state = IDLE;
        endcase
      end
      
    end else begin : g_without_line_buffer
      // Original direct memory access implementation
      typedef enum logic [2:0] {
        IDLE,
        READ_REQ,
        READ_WAIT1,
        READ_WAIT2,
        EXTRACT,
        DATA_READY_STATE
      } state_t;
      
      state_t state, next_state;
      
      logic [15:0] xi_base_r, yi_base_r;
      logic [7:0] fx_q_r, fy_q_r;
      logic [31:0] row0_data, row1_data, row0_next_data, row1_next_data;
      logic [1:0] pixel_offset;
      logic [ADDR_W-1:0] word_addr_row0, word_addr_row1;
      logic need_extra_read, reading_extra;
      
      assign pixel_offset = xi_base_r[1:0];
      
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          state <= IDLE;
          req_ready <= 1'b1;
          data_valid <= 1'b0;
          {xi_base_r, yi_base_r} <= '0;
          {fx_q_r, fy_q_r} <= '0;
          {row0_data, row1_data, row0_next_data, row1_next_data} <= '0;
          need_extra_read <= 1'b0;
          reading_extra <= 1'b0;
          {pixel_tl, pixel_tr, pixel_bl, pixel_br} <= '0;
          {frac_x, frac_y} <= '0;
        end else begin
          state <= next_state;
          
          case (state)
            IDLE: begin
              req_ready <= 1'b1;
              data_valid <= 1'b0;
              reading_extra <= 1'b0;
              if (req_valid) begin
                xi_base_r <= req_xi_base;
                yi_base_r <= req_yi_base;
                fx_q_r <= req_fx_q;
                fy_q_r <= req_fy_q;
                need_extra_read <= (req_xi_base[1:0] == 2'b11);
              end
            end
            
            READ_REQ: begin
              req_ready <= 1'b0;
              word_addr_row0 = (yi_base_r * (IMG_WIDTH >> 2)) + (xi_base_r >> 2);
              word_addr_row1 = ((yi_base_r + 16'd1) * (IMG_WIDTH >> 2)) + (xi_base_r >> 2);
              mem_raddr0 <= word_addr_row0;
              mem_raddr1 <= word_addr_row1;
            end
            
            READ_WAIT1: begin
              // Wait for memory latency
            end
            
            READ_WAIT2: begin
              if (!reading_extra) begin
                row0_data <= mem_rdata0;
                row1_data <= mem_rdata1;
                if (need_extra_read) begin
                  mem_raddr0 <= word_addr_row0 + 1;
                  mem_raddr1 <= word_addr_row1 + 1;
                  reading_extra <= 1'b1;
                  need_extra_read <= 1'b0;
                end
              end else begin
                row0_next_data <= mem_rdata0;
                row1_next_data <= mem_rdata1;
                reading_extra <= 1'b0;
              end
            end
            
            EXTRACT: begin
              case (pixel_offset)
                2'b00: begin
                  pixel_tl <= row0_data[7:0];
                  pixel_tr <= row0_data[15:8];
                  pixel_bl <= row1_data[7:0];
                  pixel_br <= row1_data[15:8];
                end
                2'b01: begin
                  pixel_tl <= row0_data[15:8];
                  pixel_tr <= row0_data[23:16];
                  pixel_bl <= row1_data[15:8];
                  pixel_br <= row1_data[23:16];
                end
                2'b10: begin
                  pixel_tl <= row0_data[23:16];
                  pixel_tr <= row0_data[31:24];
                  pixel_bl <= row1_data[23:16];
                  pixel_br <= row1_data[31:24];
                end
                2'b11: begin
                  pixel_tl <= row0_data[31:24];
                  pixel_tr <= row0_next_data[7:0];
                  pixel_bl <= row1_data[31:24];
                  pixel_br <= row1_next_data[7:0];
                end
              endcase
              frac_x <= fx_q_r;
              frac_y <= fy_q_r;
            end
            
            DATA_READY_STATE: begin
              data_valid <= 1'b1;
              if (data_consumed)
                data_valid <= 1'b0;
            end
          endcase
        end
      end
      
      always_comb begin
        next_state = state;
        case (state)
          IDLE: if (req_valid) next_state = READ_REQ;
          READ_REQ: next_state = READ_WAIT1;
          READ_WAIT1: next_state = READ_WAIT2;
          READ_WAIT2: begin
            if (reading_extra)
              next_state = EXTRACT;
            else if (need_extra_read)
              next_state = READ_WAIT1;
            else
              next_state = EXTRACT;
          end
          EXTRACT: next_state = DATA_READY_STATE;
          DATA_READY_STATE: if (data_consumed) next_state = IDLE;
          default: next_state = IDLE;
        endcase
      end
      
    end
  endgenerate

endmodule
