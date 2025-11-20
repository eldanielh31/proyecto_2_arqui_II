// ============================================================================
// top_dsa_seq.sv — Top con Virtual JTAG, LEDs y BRAMs (lectura/escritura)
// Núcleo secuencial bilinear + performance counters.
// ============================================================================
`timescale 1ps/1ps
`define MEM_INIT_FILE "img_in_64x64.hex"

module top_dsa_seq #(
  parameter int AW            = 12,
  parameter int DEB_W         = 20,
  parameter int RST_STRETCH_W = 22
)(
  input  logic clk_50,
  input  logic rst_n,
  input  logic start_sw,

  output logic led_done,
  output logic led_reset_evt,
  output logic led_start_on
);

  // ---------------- Señales core ----------------
  logic        start_pulse_sw;
  logic        start_pulse_jtag;
  logic        busy, done;

  logic [15:0] in_w_cfg, in_h_cfg, scale_q88_cfg;
  logic [15:0] in_w, in_h, scale_q88;

  logic [15:0] out_w_s, out_h_s;

  logic [AW-1:0] in_raddr_core;
  logic [7:0]    in_rdata;

  logic [AW-1:0] out_waddr;
  logic [7:0]    out_wdata;
  logic          out_we;

  // Performance counters desde el core
  logic [31:0] perf_flops;
  logic [31:0] perf_mem_rd;
  logic [31:0] perf_mem_wr;

  // Señales JTAG para BRAM (lectura/escritura)
  logic [AW-1:0] jtag_in_raddr;
  logic  [7:0]   jtag_in_rdata;
  logic [AW-1:0] jtag_out_raddr;
  logic  [7:0]   jtag_out_rdata;

  logic [AW-1:0] jtag_in_waddr;
  logic  [7:0]   jtag_in_wdata;
  logic          jtag_in_we;

  // =======================================================================
  // 1) Virtual JTAG + wrapper jtag_connect
  // =======================================================================
  logic       tck, tdi, tdo;
  logic [1:0] ir_in;
  logic       vs_cdr, vs_sdr, vs_e1dr, vs_pdr, vs_e2dr, vs_udr, vs_cir, vs_uir;

  vjtag u_vjtag (
    .tdi                (tdi),
    .tdo                (tdo),
    .ir_in              (ir_in),
    .ir_out             (),     // ir_out no se usa
    .virtual_state_cdr  (vs_cdr),
    .virtual_state_sdr  (vs_sdr),
    .virtual_state_e1dr (vs_e1dr),
    .virtual_state_pdr  (vs_pdr),
    .virtual_state_e2dr (vs_e2dr),
    .virtual_state_udr  (vs_udr),
    .virtual_state_cir  (vs_cir),
    .virtual_state_uir  (vs_uir),
    .tck                (tck)
  );

  jtag_connect #(.DRW(40), .AW(AW)) u_jc (
    .tck           (tck),
    .tdi           (tdi),
    .tdo           (tdo),
    .ir_in         (ir_in),
    .vs_cdr        (vs_cdr),
    .vs_sdr        (vs_sdr),
    .vs_udr        (vs_udr),

    .start_pulse   (start_pulse_jtag),
    .cfg_in_w      (in_w_cfg),
    .cfg_in_h      (in_h_cfg),
    .cfg_scale_q88 (scale_q88_cfg),
    .status_done   (done),
    .status_busy   (busy),
    .perf_flops    (perf_flops),
    .perf_mem_rd   (perf_mem_rd),
    .perf_mem_wr   (perf_mem_wr),

    .in_mem_raddr  (jtag_in_raddr),
    .in_mem_rdata  (jtag_in_rdata),

    .in_mem_waddr  (jtag_in_waddr),
    .in_mem_wdata  (jtag_in_wdata),
    .in_mem_we     (jtag_in_we),

    .out_mem_raddr (jtag_out_raddr),
    .out_mem_rdata (jtag_out_rdata),

    .clk_sys       (clk_50),
    .rst_sys_n     (rst_n)
  );

  // =========================================================================
  // 2) Sincronizador + antirrebote del switch de start
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
  logic             sw_debounced, sw_debounced_q;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      deb_cnt        <= '0;
      sw_debounced   <= 1'b0;
      sw_debounced_q <= 1'b0;
    end else begin
      if (sw_sync != sw_debounced) deb_cnt <= '0;
      else if (deb_cnt != {DEB_W{1'b1}}) deb_cnt <= deb_cnt + 1'b1;
      if (deb_cnt == {DEB_W{1'b1}}) sw_debounced <= sw_sync;
      sw_debounced_q <= sw_debounced;
    end
  end

  assign start_pulse_sw = (sw_debounced & ~sw_debounced_q);
  assign led_start_on   = sw_debounced;

  // =========================================================================
  // 3) LED de evento de reset
  // =========================================================================
  logic [RST_STRETCH_W-1:0] rst_cnt;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) rst_cnt <= {RST_STRETCH_W{1'b1}};
    else if (rst_cnt != '0) rst_cnt <= rst_cnt - 1'b1;
  end
  assign led_reset_evt = (rst_cnt != '0);

  // =========================================================================
  // 4) Memorias on-chip (inferidas)
  // =========================================================================
  onchip_mem_dp #(
    .ADDR_W (AW),
    .INIT_EN(1'b1)              // simulación: carga MEM_INIT_FILE (si está definido)
  ) mem_in (
    .clk   (clk_50),
    .raddr (in_raddr_core),
    .rdata (in_rdata),
    .waddr (jtag_in_waddr),
    .wdata (jtag_in_wdata),
    .we    (jtag_in_we & ~busy) // evitar escribir durante procesamiento
  );

  onchip_mem_dp #(
    .ADDR_W (AW),
    .INIT_EN(1'b1)
  ) mem_in_view (
    .clk   (clk_50),
    .raddr (jtag_in_raddr),
    .rdata (jtag_in_rdata),
    .waddr (jtag_in_waddr),
    .wdata (jtag_in_wdata),
    .we    (jtag_in_we & ~busy)
  );

  onchip_mem_dp #(
    .ADDR_W (AW),
    .INIT_EN(1'b0)
  ) mem_out (
    .clk   (clk_50),
    .raddr (jtag_out_raddr),
    .rdata (jtag_out_rdata),
    .waddr (out_waddr),
    .wdata (out_wdata),
    .we    (out_we)
  );

  // =========================================================================
  // 5) Parámetros desde JTAG
  // =========================================================================
  assign in_w      = in_w_cfg;
  assign in_h      = in_h_cfg;
  assign scale_q88 = scale_q88_cfg;

  // Start por JTAG o por switch (combinacional, sin inicialización en declaración)
  logic start_any;
  always_comb begin
    start_any = start_pulse_jtag | start_pulse_sw;
  end

  // =========================================================================
  // 6) Núcleo bilineal (secuencial)
  // =========================================================================
  bilinear_seq #(.AW(AW)) core (
    .clk           (clk_50),
    .rst_n         (rst_n),
    .start         (start_any),
    .busy          (busy),
    .done          (done),

    .i_step_en     (1'b0),   // stepping desactivado por ahora
    .i_step_pulse  (1'b0),

    .i_in_w        (in_w),
    .i_in_h        (in_h),
    .i_scale_q88   (scale_q88),

    .o_out_w       (out_w_s),
    .o_out_h       (out_h_s),

    .in_raddr      (in_raddr_core),
    .in_rdata      (in_rdata),

    .out_waddr     (out_waddr),
    .out_wdata     (out_wdata),
    .out_we        (out_we),

    .o_flop_count  (perf_flops),
    .o_mem_rd_count(perf_mem_rd),
    .o_mem_wr_count(perf_mem_wr)
  );

  // =========================================================================
  // 7) LED done latcheado
  // =========================================================================
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) led_done <= 1'b0;
    else begin
      if (start_any) led_done <= 1'b0;
      else if (done) led_done <= 1'b1;
    end
  end

endmodule
