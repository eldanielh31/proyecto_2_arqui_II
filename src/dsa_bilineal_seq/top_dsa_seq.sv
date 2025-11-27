// ============================================================================
// top_dsa_wide.sv
//   - Top module with wide memory architecture
//   - Single input memory (32-bit wide words)
//   - 2 read ports to match M10K constraints
//   - Sequential and SIMD4 cores share same wide memory
// ============================================================================
`define MEM_INIT_FILE "C:/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_16x16.hex"

module top_dsa_seq #(
  parameter int AW = 10,  // Reduced by 2 bits for wide memory
  parameter bit SIMULATION = 0,
  parameter int DEB_W = 20,
  parameter int RST_STRETCH_W = 22
)(
  input  logic clk_50,
  input  logic rst_n,

  input  logic start_sw,
  input  logic mode_simd_sw,

  output logic led_done,
  output logic led_reset_evt,
  output logic led_start_on,
  output logic led_simd_mode
);

  // Core signals
  logic start_pulse_sw;
  logic busy, done;

  logic [15:0] in_w_cfg, in_h_cfg, scale_q88_cfg;
  logic [15:0] in_w, in_h, scale_q88;

  logic mode_simd_cfg, mode_simd_eff;

  // Performance counters
  logic [31:0] perf_flops, perf_mem_rd, perf_mem_wr;

  // =========================================================================
  // Configuration (hardcoded for simulation)
  // =========================================================================
  assign in_w_cfg = 16'd16;
  assign in_h_cfg = 16'd16;
  assign scale_q88_cfg = 16'd205;  // ~0.80 in Q8.8
  assign mode_simd_cfg = 1'b0;

  // =========================================================================
  // Start switch debouncing
  // =========================================================================
  logic sw_meta, sw_sync;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) {sw_meta, sw_sync} <= 2'b00;
    else begin
      sw_meta <= start_sw;
      sw_sync <= sw_meta;
    end
  end

  logic [DEB_W-1:0] deb_cnt;
  logic sw_debounced, sw_debounced_q;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      deb_cnt <= '0;
      sw_debounced <= 1'b0;
      sw_debounced_q <= 1'b0;
    end else begin
      if (sw_sync != sw_debounced) deb_cnt <= '0;
      else if (deb_cnt != {DEB_W{1'b1}}) deb_cnt <= deb_cnt + 1'b1;
      if (deb_cnt == {DEB_W{1'b1}}) sw_debounced <= sw_sync;
      sw_debounced_q <= sw_debounced;
    end
  end

  assign start_pulse_sw = (sw_debounced & ~sw_debounced_q);
  assign led_start_on = sw_debounced;

  // =========================================================================
  // Reset event LED
  // =========================================================================
  logic [RST_STRETCH_W-1:0] rst_cnt;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) rst_cnt <= {RST_STRETCH_W{1'b1}};
    else if (rst_cnt != '0) rst_cnt <= rst_cnt - 1'b1;
  end
  assign led_reset_evt = (rst_cnt != '0);

  // =========================================================================
  // Wide memory instances
  // =========================================================================
  
  // Input memory (wide, 2 read ports)
  logic [AW-1:0] mem_in_raddr0, mem_in_raddr1;
  logic [31:0]   mem_in_rdata0, mem_in_rdata1;
  
  wide_onchip_mem #(
    .ADDR_W(AW),
    .INIT_EN(1'b1)
  ) mem_in (
    .clk(clk_50),
    .raddr0(mem_in_raddr0),
    .rdata0(mem_in_rdata0),
    .raddr1(mem_in_raddr1),
    .rdata1(mem_in_rdata1),
    .waddr0('0), .wdata0(8'h00), .we0(1'b0),
    .waddr1('0), .wdata1(8'h00), .we1(1'b0),
    .waddr2('0), .wdata2(8'h00), .we2(1'b0),
    .waddr3('0), .wdata3(8'h00), .we3(1'b0)
  );

  // Output memory (wide, 4 write ports)
  logic [AW-1:0] mem_out_waddr0, mem_out_waddr1, mem_out_waddr2, mem_out_waddr3;
  logic [7:0]    mem_out_wdata0, mem_out_wdata1, mem_out_wdata2, mem_out_wdata3;
  logic          mem_out_we0, mem_out_we1, mem_out_we2, mem_out_we3;
  
  wide_onchip_mem #(
    .ADDR_W(AW),
    .INIT_EN(1'b0)
  ) mem_out (
    .clk(clk_50),
    .raddr0('0), .rdata0(),
    .raddr1('0), .rdata1(),
    .waddr0(mem_out_waddr0),
    .wdata0(mem_out_wdata0),
    .we0(mem_out_we0),
    .waddr1(mem_out_waddr1),
    .wdata1(mem_out_wdata1),
    .we1(mem_out_we1),
    .waddr2(mem_out_waddr2),
    .wdata2(mem_out_wdata2),
    .we2(mem_out_we2),
    .waddr3(mem_out_waddr3),
    .wdata3(mem_out_wdata3),
    .we3(mem_out_we3)
  );

  // =========================================================================
  // Configuration
  // =========================================================================
  assign in_w = in_w_cfg;
  assign in_h = in_h_cfg;
  assign scale_q88 = scale_q88_cfg;
  assign mode_simd_eff = mode_simd_cfg | mode_simd_sw;
  assign led_simd_mode = mode_simd_eff;

  // =========================================================================
  // Output memory clear
  // =========================================================================
  localparam int OUT_DEPTH = (1 << AW);
  logic clear_active;
  logic [AW-1:0] clear_addr;

  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      clear_active <= 1'b1;
      clear_addr <= {AW{1'b0}};
    end else begin
      if (clear_active) begin
        clear_addr <= clear_addr + 1'b1;
        if (clear_addr == OUT_DEPTH - 1)
          clear_active <= 1'b0;
      end
    end
  end

  // =========================================================================
  // Start control
  // =========================================================================
  logic start_raw;
  logic start_any;

  assign start_raw = (SIMULATION) ? start_sw : start_pulse_sw;
  assign start_any = start_raw & ~clear_active;

  // =========================================================================
  // Core instances
  // =========================================================================
  
  // Sequential core signals
  logic busy_seq, done_seq;
  logic [AW-1:0] in_raddr0_seq, in_raddr1_seq;
  logic [31:0] in_rdata0_seq, in_rdata1_seq;
  logic [AW-1:0] out_waddr_seq;
  logic [7:0] out_wdata_seq;
  logic out_we_seq;
  logic [31:0] perf_flops_seq, perf_mem_rd_seq, perf_mem_wr_seq;

  // SIMD4 core signals
  logic busy_simd, done_simd;
  logic [AW-1:0] in_raddr0_simd, in_raddr1_simd;
  logic [31:0] in_rdata0_simd, in_rdata1_simd;
  logic [AW-1:0] out_waddr_simd0, out_waddr_simd1, out_waddr_simd2, out_waddr_simd3;
  logic [7:0] out_wdata_simd0, out_wdata_simd1, out_wdata_simd2, out_wdata_simd3;
  logic out_we_simd0, out_we_simd1, out_we_simd2, out_we_simd3;
  logic [31:0] perf_flops_simd, perf_mem_rd_simd, perf_mem_wr_simd;

  logic start_seq, start_simd;
  assign start_seq = start_any & ~mode_simd_eff;
  assign start_simd = start_any & mode_simd_eff;

  // Sequential core
  bilinear_seq_wide #(.AW(AW)) u_core_seq (
    .clk(clk_50),
    .rst_n(rst_n),
    .start(start_seq),
    .busy(busy_seq),
    .done(done_seq),
    .i_step_en(1'b0),
    .i_step_pulse(1'b0),
    .i_in_w(in_w),
    .i_in_h(in_h),
    .i_scale_q88(scale_q88),
    .o_out_w(),
    .o_out_h(),
    .in_raddr0(in_raddr0_seq),
    .in_rdata0(in_rdata0_seq),
    .in_raddr1(in_raddr1_seq),
    .in_rdata1(in_rdata1_seq),
    .out_waddr(out_waddr_seq),
    .out_wdata(out_wdata_seq),
    .out_we(out_we_seq),
    .o_flop_count(perf_flops_seq),
    .o_mem_rd_count(perf_mem_rd_seq),
    .o_mem_wr_count(perf_mem_wr_seq)
  );

  // SIMD4 core
  bilinear_simd4_wide #(.AW(AW)) u_core_simd4 (
    .clk(clk_50),
    .rst_n(rst_n),
    .start(start_simd),
    .busy(busy_simd),
    .done(done_simd),
    .i_step_en(1'b0),
    .i_step_pulse(1'b0),
    .i_in_w(in_w),
    .i_in_h(in_h),
    .i_scale_q88(scale_q88),
    .o_out_w(),
    .o_out_h(),
    .in_raddr0(in_raddr0_simd),
    .in_rdata0(in_rdata0_simd),
    .in_raddr1(in_raddr1_simd),
    .in_rdata1(in_rdata1_simd),
    .out_waddr0(out_waddr_simd0),
    .out_wdata0(out_wdata_simd0),
    .out_we0(out_we_simd0),
    .out_waddr1(out_waddr_simd1),
    .out_wdata1(out_wdata_simd1),
    .out_we1(out_we_simd1),
    .out_waddr2(out_waddr_simd2),
    .out_wdata2(out_wdata_simd2),
    .out_we2(out_we_simd2),
    .out_waddr3(out_waddr_simd3),
    .out_wdata3(out_wdata_simd3),
    .out_we3(out_we_simd3),
    .o_flop_count(perf_flops_simd),
    .o_mem_rd_count(perf_mem_rd_simd),
    .o_mem_wr_count(perf_mem_wr_simd)
  );

  // =========================================================================
  // Memory read mux
  // =========================================================================
  always_comb begin
    if (mode_simd_eff) begin
      mem_in_raddr0 = in_raddr0_simd;
      mem_in_raddr1 = in_raddr1_simd;
      in_rdata0_simd = mem_in_rdata0;
      in_rdata1_simd = mem_in_rdata1;
      in_rdata0_seq = 32'h0;
      in_rdata1_seq = 32'h0;
    end else begin
      mem_in_raddr0 = in_raddr0_seq;
      mem_in_raddr1 = in_raddr1_seq;
      in_rdata0_seq = mem_in_rdata0;
      in_rdata1_seq = mem_in_rdata1;
      in_rdata0_simd = 32'h0;
      in_rdata1_simd = 32'h0;
    end
  end

  // =========================================================================
  // Memory write mux
  // =========================================================================
  always_comb begin
    mem_out_waddr0 = '0; mem_out_wdata0 = 8'h00; mem_out_we0 = 1'b0;
    mem_out_waddr1 = '0; mem_out_wdata1 = 8'h00; mem_out_we1 = 1'b0;
    mem_out_waddr2 = '0; mem_out_wdata2 = 8'h00; mem_out_we2 = 1'b0;
    mem_out_waddr3 = '0; mem_out_wdata3 = 8'h00; mem_out_we3 = 1'b0;

    if (clear_active) begin
      mem_out_waddr0 = clear_addr;
      mem_out_wdata0 = 8'h00;
      mem_out_we0 = 1'b1;
    end else if (mode_simd_eff) begin
      mem_out_waddr0 = out_waddr_simd0;
      mem_out_wdata0 = out_wdata_simd0;
      mem_out_we0 = out_we_simd0;
      mem_out_waddr1 = out_waddr_simd1;
      mem_out_wdata1 = out_wdata_simd1;
      mem_out_we1 = out_we_simd1;
      mem_out_waddr2 = out_waddr_simd2;
      mem_out_wdata2 = out_wdata_simd2;
      mem_out_we2 = out_we_simd2;
      mem_out_waddr3 = out_waddr_simd3;
      mem_out_wdata3 = out_wdata_simd3;
      mem_out_we3 = out_we_simd3;
    end else begin
      mem_out_waddr0 = out_waddr_seq;
      mem_out_wdata0 = out_wdata_seq;
      mem_out_we0 = out_we_seq;
    end
  end

  // =========================================================================
  // Status outputs
  // =========================================================================
  assign busy = mode_simd_eff ? busy_simd : busy_seq;
  assign done = mode_simd_eff ? done_simd : done_seq;
  assign perf_flops = mode_simd_eff ? perf_flops_simd : perf_flops_seq;
  assign perf_mem_rd = mode_simd_eff ? perf_mem_rd_simd : perf_mem_rd_seq;
  assign perf_mem_wr = mode_simd_eff ? perf_mem_wr_simd : perf_mem_wr_seq;

  // =========================================================================
  // LED done
  // =========================================================================
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) led_done <= 1'b0;
    else begin
      if (start_any) led_done <= 1'b0;
      else if (done) led_done <= 1'b1;
    end
  end

endmodule