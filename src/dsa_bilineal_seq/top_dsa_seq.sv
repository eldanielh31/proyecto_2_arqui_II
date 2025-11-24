	`timescale 1ps/1ps
	`define MEM_INIT_FILE "img_in_64x64.hex"

	// ============================================================================
	// top_dsa_seq.sv — Top con Virtual JTAG (deshabilitado en sim), LEDs y BRAMs.
	// Núcleo secuencial + núcleo SIMD4, seleccionables por modo (switch).
	//
	// onchip_mem_dp: 4R/4W
	//
	// Nueva organización de memoria de entrada:
	//   - 4 bancos idénticos (mem_in, mem_in1, mem_in2, mem_in3)
	//   - Cada banco almacena la misma imagen (mismo MEM_INIT_FILE).
	//   - bilinear_seq usa solo mem_in (banco 0, puerto 0).
	//   - bilinear_simd4 usa los 4 bancos, 4 puertos por banco
	//     para obtener 16 bytes (4 vecinos × 4 lanes) por grupo SIMD.
	// ============================================================================

	module top_dsa_seq #(
	  parameter int AW            = 12,
	  parameter int DEB_W         = 20,
	  parameter int RST_STRETCH_W = 22
	)(
	  input  logic clk_50,
	  input  logic rst_n,

	  // Switch físico de start
	  input  logic start_sw,

	  // Switch físico para modo SIMD x4
	  input  logic mode_simd_sw,

	  // LEDs
	  output logic led_done,
	  output logic led_reset_evt,
	  output logic led_start_on,
	  output logic led_simd_mode
	);

	  // ---------------- Señales core (comunes) ----------------
	  logic        start_pulse_sw;
	  logic        start_pulse_jtag;

	  logic        busy;
	  logic        done;

	  logic [15:0] in_w_cfg, in_h_cfg, scale_q88_cfg;
	  logic [15:0] in_w, in_h, scale_q88;

	  logic        mode_simd_cfg;
	  logic        mode_simd_eff;

	  // Performance counters globales
	  logic [31:0] perf_flops;
	  logic [31:0] perf_mem_rd;
	  logic [31:0] perf_mem_wr;

	  // Señales JTAG (dummy en sim)
	  logic [AW-1:0] jtag_in_raddr;
	  logic  [7:0]   jtag_in_rdata;
	  logic [AW-1:0] jtag_out_raddr;
	  logic  [7:0]   jtag_out_rdata;

	  logic [AW-1:0] jtag_in_waddr;
	  logic  [7:0]   jtag_in_wdata;
	  logic          jtag_in_we;

	  // =========================================================================
	  // SIM ONLY: deshabilitar JTAG, usar constantes internas
	  // =========================================================================

	  assign start_pulse_jtag = 1'b0;

	  assign in_w_cfg      = 16'd64;
	  assign in_h_cfg      = 16'd64;
	  assign scale_q88_cfg = 16'd205;   // ≈0.80 en Q8.8

	  assign mode_simd_cfg   = 1'b0;

	  assign jtag_in_raddr   = '0;
	  assign jtag_out_raddr  = '0;
	  assign jtag_in_waddr   = '0;
	  assign jtag_in_wdata   = 8'h00;
	  assign jtag_in_we      = 1'b0;

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
	  // 4) Memorias on-chip (inferidas) — 4 bancos de entrada + mem_out
	  // =========================================================================

	  // -----------------------
	  // Banco 0: mem_in (lane0)
	  // -----------------------
	  // Señales hacia mem_in (4R)
	  logic [AW-1:0] mem_in_raddr0, mem_in_raddr1, mem_in_raddr2, mem_in_raddr3;
	  logic [7:0]    mem_in_rdata0, mem_in_rdata1, mem_in_rdata2, mem_in_rdata3;

	  // mem_in — BRAM de entrada principal (banco 0)
	  onchip_mem_dp #(
		 .ADDR_W (AW),
		 .INIT_EN(1'b1)
	  ) mem_in (
		 .clk    (clk_50),

		 .raddr0 (mem_in_raddr0),
		 .rdata0 (mem_in_rdata0),

		 .raddr1 (mem_in_raddr1),
		 .rdata1 (mem_in_rdata1),

		 .raddr2 (mem_in_raddr2),
		 .rdata2 (mem_in_rdata2),

		 .raddr3 (mem_in_raddr3),
		 .rdata3 (mem_in_rdata3),

		 .waddr0 (jtag_in_waddr),
		 .wdata0 (jtag_in_wdata),
		 .we0    (jtag_in_we & ~busy),

		 .waddr1 ('0),
		 .wdata1 (8'h00),
		 .we1    (1'b0),

		 .waddr2 ('0),
		 .wdata2 (8'h00),
		 .we2    (1'b0),

		 .waddr3 ('0),
		 .wdata3 (8'h00),
		 .we3    (1'b0)
	  );

	  // -----------------------
	  // Banco 1: mem_in1 (lane1)
	  // -----------------------
	  logic [AW-1:0] mem_in1_raddr0, mem_in1_raddr1, mem_in1_raddr2, mem_in1_raddr3;
	  logic [7:0]    mem_in1_rdata0, mem_in1_rdata1, mem_in1_rdata2, mem_in1_rdata3;

	  onchip_mem_dp #(
		 .ADDR_W (AW),
		 .INIT_EN(1'b1)
	  ) mem_in1 (
		 .clk    (clk_50),

		 .raddr0 (mem_in1_raddr0),
		 .rdata0 (mem_in1_rdata0),

		 .raddr1 (mem_in1_raddr1),
		 .rdata1 (mem_in1_rdata1),

		 .raddr2 (mem_in1_raddr2),
		 .rdata2 (mem_in1_rdata2),

		 .raddr3 (mem_in1_raddr3),
		 .rdata3 (mem_in1_rdata3),

		 .waddr0 (jtag_in_waddr),
		 .wdata0 (jtag_in_wdata),
		 .we0    (jtag_in_we & ~busy),

		 .waddr1 ('0),
		 .wdata1 (8'h00),
		 .we1    (1'b0),

		 .waddr2 ('0),
		 .wdata2 (8'h00),
		 .we2    (1'b0),

		 .waddr3 ('0),
		 .wdata3 (8'h00),
		 .we3    (1'b0)
	  );

	  // -----------------------
	  // Banco 2: mem_in2 (lane2)
	  // -----------------------
	  logic [AW-1:0] mem_in2_raddr0, mem_in2_raddr1, mem_in2_raddr2, mem_in2_raddr3;
	  logic [7:0]    mem_in2_rdata0, mem_in2_rdata1, mem_in2_rdata2, mem_in2_rdata3;

	  onchip_mem_dp #(
		 .ADDR_W (AW),
		 .INIT_EN(1'b1)
	  ) mem_in2 (
		 .clk    (clk_50),

		 .raddr0 (mem_in2_raddr0),
		 .rdata0 (mem_in2_rdata0),

		 .raddr1 (mem_in2_raddr1),
		 .rdata1 (mem_in2_rdata1),

		 .raddr2 (mem_in2_raddr2),
		 .rdata2 (mem_in2_rdata2),

		 .raddr3 (mem_in2_raddr3),
		 .rdata3 (mem_in2_rdata3),

		 .waddr0 (jtag_in_waddr),
		 .wdata0 (jtag_in_wdata),
		 .we0    (jtag_in_we & ~busy),

		 .waddr1 ('0),
		 .wdata1 (8'h00),
		 .we1    (1'b0),

		 .waddr2 ('0),
		 .wdata2 (8'h00),
		 .we2    (1'b0),

		 .waddr3 ('0),
		 .wdata3 (8'h00),
		 .we3    (1'b0)
	  );

	  // -----------------------
	  // Banco 3: mem_in3 (lane3)
	  // -----------------------
	  logic [AW-1:0] mem_in3_raddr0, mem_in3_raddr1, mem_in3_raddr2, mem_in3_raddr3;
	  logic [7:0]    mem_in3_rdata0, mem_in3_rdata1, mem_in3_rdata2, mem_in3_rdata3;

	  onchip_mem_dp #(
		 .ADDR_W (AW),
		 .INIT_EN(1'b1)
	  ) mem_in3 (
		 .clk    (clk_50),

		 .raddr0 (mem_in3_raddr0),
		 .rdata0 (mem_in3_rdata0),

		 .raddr1 (mem_in3_raddr1),
		 .rdata1 (mem_in3_rdata1),

		 .raddr2 (mem_in3_raddr2),
		 .rdata2 (mem_in3_rdata2),

		 .raddr3 (mem_in3_raddr3),
		 .rdata3 (mem_in3_rdata3),

		 .waddr0 (jtag_in_waddr),
		 .wdata0 (jtag_in_wdata),
		 .we0    (jtag_in_we & ~busy),

		 .waddr1 ('0),
		 .wdata1 (8'h00),
		 .we1    (1'b0),

		 .waddr2 ('0),
		 .wdata2 (8'h00),
		 .we2    (1'b0),

		 .waddr3 ('0),
		 .wdata3 (8'h00),
		 .we3    (1'b0)
	  );

	  // mem_in_view — copia solo para lectura por JTAG (sin cambios)
	  onchip_mem_dp #(
		 .ADDR_W (AW),
		 .INIT_EN(1'b1)
	  ) mem_in_view (
		 .clk    (clk_50),

		 .raddr0 (jtag_in_raddr),
		 .rdata0 (jtag_in_rdata),

		 .raddr1 ('0),
		 .rdata1 (),

		 .raddr2 ('0),
		 .rdata2 (),

		 .raddr3 ('0),
		 .rdata3 (),

		 .waddr0 (jtag_in_waddr),
		 .wdata0 (jtag_in_wdata),
		 .we0    (jtag_in_we & ~busy),

		 .waddr1 ('0),
		 .wdata1 (8'h00),
		 .we1    (1'b0),

		 .waddr2 ('0),
		 .wdata2 (8'h00),
		 .we2    (1'b0),

		 .waddr3 ('0),
		 .wdata3 (8'h00),
		 .we3    (1'b0)
	  );

	  // Señales de escritura a mem_out (4W)
	  logic [AW-1:0] mem_out_waddr0, mem_out_waddr1, mem_out_waddr2, mem_out_waddr3;
	  logic [7:0]    mem_out_wdata0, mem_out_wdata1, mem_out_wdata2, mem_out_wdata3;
	  logic          mem_out_we0,    mem_out_we1,    mem_out_we2,    mem_out_we3;

	  // mem_out — salida
	  onchip_mem_dp #(
		 .ADDR_W (AW),
		 .INIT_EN(1'b0)
	  ) mem_out (
		 .clk    (clk_50),

		 .raddr0 (jtag_out_raddr),
		 .rdata0 (jtag_out_rdata),

		 .raddr1 ('0),
		 .rdata1 (),

		 .raddr2 ('0),
		 .rdata2 (),

		 .raddr3 ('0),
		 .rdata3 (),

		 .waddr0 (mem_out_waddr0),
		 .wdata0 (mem_out_wdata0),
		 .we0    (mem_out_we0),

		 .waddr1 (mem_out_waddr1),
		 .wdata1 (mem_out_wdata1),
		 .we1    (mem_out_we1),

		 .waddr2 (mem_out_waddr2),
		 .wdata2 (mem_out_wdata2),
		 .we2    (mem_out_we2),

		 .waddr3 (mem_out_waddr3),
		 .wdata3 (mem_out_wdata3),
		 .we3    (mem_out_we3)
	  );

	  // =========================================================================
	  // 5) Parámetros desde JTAG (aquí fijos)
	  // =========================================================================
	  assign in_w      = in_w_cfg;
	  assign in_h      = in_h_cfg;
	  assign scale_q88 = scale_q88_cfg;

	  assign mode_simd_eff = mode_simd_cfg | mode_simd_sw;
	  assign led_simd_mode = mode_simd_eff;

	  // =========================================================================
	  // 6) Limpieza de mem_out tras reset
	  // =========================================================================
	  localparam int OUT_DEPTH = (1 << AW);

	  logic          clear_active;
	  logic [AW-1:0] clear_addr;

	  always_ff @(posedge clk_50 or negedge rst_n) begin
		 if (!rst_n) begin
			clear_active <= 1'b1;
			clear_addr   <= {AW{1'b0}};
		 end else begin
			if (clear_active) begin
			  clear_addr <= clear_addr + 1'b1;
			  if (clear_addr == OUT_DEPTH-1)
				 clear_active <= 1'b0;
			end
		 end
	  end

	  // =========================================================================
	  // 7) Start global
	  // =========================================================================
	  logic start_any_raw, start_any;

	  always_comb begin
		 start_any_raw = start_pulse_jtag | start_pulse_sw;
		 start_any     = start_any_raw & ~clear_active;
	  end

	  // =========================================================================
	  // 8) Núcleos bilineales
	  // =========================================================================
	  // Secuencial
	  logic          busy_seq,  done_seq;
	  logic [AW-1:0] out_waddr_seq;
	  logic [7:0]    out_wdata_seq;
	  logic          out_we_seq;
	  logic [31:0]   perf_flops_seq, perf_mem_rd_seq, perf_mem_wr_seq;
	  logic [15:0]   out_w_s_seq, out_h_s_seq;

	  // SIMD4
	  logic          busy_simd, done_simd;
	  logic [AW-1:0] out_waddr_simd0, out_waddr_simd1, out_waddr_simd2, out_waddr_simd3;
	  logic [7:0]    out_wdata_simd0, out_wdata_simd1, out_wdata_simd2, out_wdata_simd3;
	  logic          out_we_simd0,    out_we_simd1,    out_we_simd2,    out_we_simd3;
	  logic [31:0]   perf_flops_simd, perf_mem_rd_simd, perf_mem_wr_simd;
	  logic [15:0]   out_w_s_simd, out_h_s_simd;

	  logic start_seq, start_simd;
	  assign start_seq  = start_any & ~mode_simd_eff;
	  assign start_simd = start_any &  mode_simd_eff;

	  // Selección de contadores y estado global
	  assign busy        = mode_simd_eff ? busy_simd        : busy_seq;
	  assign done        = mode_simd_eff ? done_simd        : done_seq;
	  assign perf_flops  = mode_simd_eff ? perf_flops_simd  : perf_flops_seq;
	  assign perf_mem_rd = mode_simd_eff ? perf_mem_rd_simd : perf_mem_rd_seq;
	  assign perf_mem_wr = mode_simd_eff ? perf_mem_wr_simd : perf_mem_wr_seq;

	  // ---------------------------
	  // Señales lectura para SEQ
	  // ---------------------------
	  logic [AW-1:0] in_raddr_seq;
	  logic [7:0]    in_rdata_seq;

	  // ---------------------------
	  // Señales lectura para SIMD4
	  // ---------------------------
	  // Lane 0
	  logic [AW-1:0] in_raddr_lane0_0, in_raddr_lane0_1, in_raddr_lane0_2, in_raddr_lane0_3;
	  logic [7:0]    in_rdata_lane0_0, in_rdata_lane0_1, in_rdata_lane0_2, in_rdata_lane0_3;

	  // Lane 1
	  logic [AW-1:0] in_raddr_lane1_0, in_raddr_lane1_1, in_raddr_lane1_2, in_raddr_lane1_3;
	  logic [7:0]    in_rdata_lane1_0, in_rdata_lane1_1, in_rdata_lane1_2, in_rdata_lane1_3;

	  // Lane 2
	  logic [AW-1:0] in_raddr_lane2_0, in_raddr_lane2_1, in_raddr_lane2_2, in_raddr_lane2_3;
	  logic [7:0]    in_rdata_lane2_0, in_rdata_lane2_1, in_rdata_lane2_2, in_rdata_lane2_3;

	  // Lane 3
	  logic [AW-1:0] in_raddr_lane3_0, in_raddr_lane3_1, in_raddr_lane3_2, in_raddr_lane3_3;
	  logic [7:0]    in_rdata_lane3_0, in_rdata_lane3_1, in_rdata_lane3_2, in_rdata_lane3_3;

	  // Núcleo secuencial
	  bilinear_seq #(.AW(AW)) u_core_seq (
		 .clk           (clk_50),
		 .rst_n         (rst_n),
		 .start         (start_seq),
		 .busy          (busy_seq),
		 .done          (done_seq),

		 .i_step_en     (1'b0),
		 .i_step_pulse  (1'b0),

		 .i_in_w        (in_w),
		 .i_in_h        (in_h),
		 .i_scale_q88   (scale_q88),

		 .o_out_w       (out_w_s_seq),
		 .o_out_h       (out_h_s_seq),

		 .in_raddr      (in_raddr_seq),
		 .in_rdata      (in_rdata_seq),

		 .out_waddr     (out_waddr_seq),
		 .out_wdata     (out_wdata_seq),
		 .out_we        (out_we_seq),

		 .o_flop_count  (perf_flops_seq),
		 .o_mem_rd_count(perf_mem_rd_seq),
		 .o_mem_wr_count(perf_mem_wr_seq)
	  );

	  // Núcleo SIMD4 — interfaz 4×4R/4W
	  bilinear_simd4 #(.AW(AW)) u_core_simd4 (
		 .clk           (clk_50),
		 .rst_n         (rst_n),
		 .start         (start_simd),
		 .busy          (busy_simd),
		 .done          (done_simd),

		 .i_step_en     (1'b0),
		 .i_step_pulse  (1'b0),

		 .i_in_w        (in_w),
		 .i_in_h        (in_h),
		 .i_scale_q88   (scale_q88),

		 .o_out_w       (out_w_s_simd),
		 .o_out_h       (out_h_s_simd),

		 // Lectura lane 0 (banco mem_in)
		 .in_raddr_lane0_0 (in_raddr_lane0_0),
		 .in_raddr_lane0_1 (in_raddr_lane0_1),
		 .in_raddr_lane0_2 (in_raddr_lane0_2),
		 .in_raddr_lane0_3 (in_raddr_lane0_3),
		 .in_rdata_lane0_0 (in_rdata_lane0_0),
		 .in_rdata_lane0_1 (in_rdata_lane0_1),
		 .in_rdata_lane0_2 (in_rdata_lane0_2),
		 .in_rdata_lane0_3 (in_rdata_lane0_3),

		 // Lectura lane 1 (banco mem_in1)
		 .in_raddr_lane1_0 (in_raddr_lane1_0),
		 .in_raddr_lane1_1 (in_raddr_lane1_1),
		 .in_raddr_lane1_2 (in_raddr_lane1_2),
		 .in_raddr_lane1_3 (in_raddr_lane1_3),
		 .in_rdata_lane1_0 (in_rdata_lane1_0),
		 .in_rdata_lane1_1 (in_rdata_lane1_1),
		 .in_rdata_lane1_2 (in_rdata_lane1_2),
		 .in_rdata_lane1_3 (in_rdata_lane1_3),

		 // Lectura lane 2 (banco mem_in2)
		 .in_raddr_lane2_0 (in_raddr_lane2_0),
		 .in_raddr_lane2_1 (in_raddr_lane2_1),
		 .in_raddr_lane2_2 (in_raddr_lane2_2),
		 .in_raddr_lane2_3 (in_raddr_lane2_3),
		 .in_rdata_lane2_0 (in_rdata_lane2_0),
		 .in_rdata_lane2_1 (in_rdata_lane2_1),
		 .in_rdata_lane2_2 (in_rdata_lane2_2),
		 .in_rdata_lane2_3 (in_rdata_lane2_3),

		 // Lectura lane 3 (banco mem_in3)
		 .in_raddr_lane3_0 (in_raddr_lane3_0),
		 .in_raddr_lane3_1 (in_raddr_lane3_1),
		 .in_raddr_lane3_2 (in_raddr_lane3_2),
		 .in_raddr_lane3_3 (in_raddr_lane3_3),
		 .in_rdata_lane3_0 (in_rdata_lane3_0),
		 .in_rdata_lane3_1 (in_rdata_lane3_1),
		 .in_rdata_lane3_2 (in_rdata_lane3_2),
		 .in_rdata_lane3_3 (in_rdata_lane3_3),

		 // Escritura 4W
		 .out_waddr0    (out_waddr_simd0),
		 .out_wdata0    (out_wdata_simd0),
		 .out_we0       (out_we_simd0),

		 .out_waddr1    (out_waddr_simd1),
		 .out_wdata1    (out_wdata_simd1),
		 .out_we1       (out_we_simd1),

		 .out_waddr2    (out_waddr_simd2),
		 .out_wdata2    (out_wdata_simd2),
		 .out_we2       (out_we_simd2),

		 .out_waddr3    (out_waddr_simd3),
		 .out_wdata3    (out_wdata_simd3),
		 .out_we3       (out_we_simd3),

		 .o_flop_count  (perf_flops_simd),
		 .o_mem_rd_count(perf_mem_rd_simd),
		 .o_mem_wr_count(perf_mem_wr_simd)
	  );

	  // =========================================================================
	  // Conexión mem_in* <-> núcleos
	  // =========================================================================

	  // Banco 0: mem_in
	  // - Modo SEQ: puerto 0 = bilinear_seq, puertos 1..3 sin uso
	  // - Modo SIMD: puertos 0..3 = lane 0 de bilinear_simd4
	  always_comb begin
		 if (mode_simd_eff) begin
			// SIMD: lane 0 usa los 4 puertos del banco 0
			mem_in_raddr0 = in_raddr_lane0_0;
			mem_in_raddr1 = in_raddr_lane0_1;
			mem_in_raddr2 = in_raddr_lane0_2;
			mem_in_raddr3 = in_raddr_lane0_3;
		 end else begin
			// SEQ: solo puerto 0
			mem_in_raddr0 = in_raddr_seq;
			mem_in_raddr1 = '0;
			mem_in_raddr2 = '0;
			mem_in_raddr3 = '0;
		 end
	  end

	  // Datos banco 0
	  assign in_rdata_seq      = mem_in_rdata0;
	  assign in_rdata_lane0_0  = mem_in_rdata0;
	  assign in_rdata_lane0_1  = mem_in_rdata1;
	  assign in_rdata_lane0_2  = mem_in_rdata2;
	  assign in_rdata_lane0_3  = mem_in_rdata3;

	  // Banco 1 -> lane 1
	  assign mem_in1_raddr0 = in_raddr_lane1_0;
	  assign mem_in1_raddr1 = in_raddr_lane1_1;
	  assign mem_in1_raddr2 = in_raddr_lane1_2;
	  assign mem_in1_raddr3 = in_raddr_lane1_3;

	  assign in_rdata_lane1_0 = mem_in1_rdata0;
	  assign in_rdata_lane1_1 = mem_in1_rdata1;
	  assign in_rdata_lane1_2 = mem_in1_rdata2;
	  assign in_rdata_lane1_3 = mem_in1_rdata3;

	  // Banco 2 -> lane 2
	  assign mem_in2_raddr0 = in_raddr_lane2_0;
	  assign mem_in2_raddr1 = in_raddr_lane2_1;
	  assign mem_in2_raddr2 = in_raddr_lane2_2;
	  assign mem_in2_raddr3 = in_raddr_lane2_3;

	  assign in_rdata_lane2_0 = mem_in2_rdata0;
	  assign in_rdata_lane2_1 = mem_in2_rdata1;
	  assign in_rdata_lane2_2 = mem_in2_rdata2;
	  assign in_rdata_lane2_3 = mem_in2_rdata3;

	  // Banco 3 -> lane 3
	  assign mem_in3_raddr0 = in_raddr_lane3_0;
	  assign mem_in3_raddr1 = in_raddr_lane3_1;
	  assign mem_in3_raddr2 = in_raddr_lane3_2;
	  assign mem_in3_raddr3 = in_raddr_lane3_3;

	  assign in_rdata_lane3_0 = mem_in3_rdata0;
	  assign in_rdata_lane3_1 = mem_in3_rdata1;
	  assign in_rdata_lane3_2 = mem_in3_rdata2;
	  assign in_rdata_lane3_3 = mem_in3_rdata3;

	  // =========================================================================
	  // 9) Mux de escritura a mem_out (clear / SEQ / SIMD)
	  // =========================================================================
	  always_comb begin
		 // Defaults (no escritura)
		 mem_out_waddr0 = '0;    mem_out_wdata0 = 8'h00; mem_out_we0 = 1'b0;
		 mem_out_waddr1 = '0;    mem_out_wdata1 = 8'h00; mem_out_we1 = 1'b0;
		 mem_out_waddr2 = '0;    mem_out_wdata2 = 8'h00; mem_out_we2 = 1'b0;
		 mem_out_waddr3 = '0;    mem_out_wdata3 = 8'h00; mem_out_we3 = 1'b0;

		 if (clear_active) begin
			// Limpieza: solo puerto 0, recorre toda la RAM
			mem_out_waddr0 = clear_addr;
			mem_out_wdata0 = 8'h00;
			mem_out_we0    = 1'b1;
		 end else if (mode_simd_eff) begin
			// Modo SIMD: hasta 4W desde el núcleo SIMD4
			mem_out_waddr0 = out_waddr_simd0;
			mem_out_wdata0 = out_wdata_simd0;
			mem_out_we0    = out_we_simd0;

			mem_out_waddr1 = out_waddr_simd1;
			mem_out_wdata1 = out_wdata_simd1;
			mem_out_we1    = out_we_simd1;

			mem_out_waddr2 = out_waddr_simd2;
			mem_out_wdata2 = out_wdata_simd2;
			mem_out_we2    = out_we_simd2;

			mem_out_waddr3 = out_waddr_simd3;
			mem_out_wdata3 = out_wdata_simd3;
			mem_out_we3    = out_we_simd3;
		 end else begin
			// Modo secuencial: solo puerto 0
			mem_out_waddr0 = out_waddr_seq;
			mem_out_wdata0 = out_wdata_seq;
			mem_out_we0    = out_we_seq;
		 end
	  end

	  // =========================================================================
	  // 10) LED done latcheado
	  // =========================================================================
	  always_ff @(posedge clk_50 or negedge rst_n) begin
		 if (!rst_n) led_done <= 1'b0;
		 else begin
			if (start_any) led_done <= 1'b0;
			else if (done) led_done <= 1'b1;
		 end
	  end

	endmodule
