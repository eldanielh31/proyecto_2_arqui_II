// ============================================================================
// top_dsa_seq.sv — Top con Virtual JTAG, LEDs y BRAMs (lectura/escritura)
// Núcleo secuencial bilinear + núcleo "SIMD" con N lanes + performance counters.
// ============================================================================
`timescale 1ps/1ps
`define MEM_INIT_FILE "img_in_64x64.hex"

module top_dsa_seq #(
  parameter int AW            = 12,
  parameter int DEB_W         = 20,
  parameter int RST_STRETCH_W = 22,
  parameter int LANES_SIMD    = 4   // N lanes lógicos del núcleo SIMD (N>=4)
)(
  input  logic clk_50,
  input  logic rst_n,
  input  logic start_sw,       // switch/botón de start
  input  logic mode_simd_sw,   // switch físico para seleccionar modo SIMD

  output logic led_done,
  output logic led_reset_evt,
  output logic led_start_on,
  output logic led_mode_simd   // LED indica que el modo SIMD está activo
);

  // ---------------- Señales core ----------------
  logic        start_pulse_sw;
  logic        start_pulse_jtag;
  logic        busy_seq, done_seq;
  logic        busy_simd, done_simd;
  logic        busy_any, done_any;

  logic        mode_simd_jtag;
  logic        mode_simd;      // modo efectivo (HW switch OR JTAG)

  logic [15:0] in_w_cfg, in_h_cfg, scale_q88_cfg;
  logic [15:0] in_w, in_h, scale_q88;

  logic [15:0] out_w_s, out_h_s;  // salidas del secuencial (no usadas, pero expuestas)

  // Direcciones de lectura de BRAM entrada para cada core
  logic [AW-1:0] in_raddr_seq;
  logic [AW-1:0] in_raddr_simd;
  logic [AW-1:0] in_raddr_core;   // hacia mem_in
  logic [7:0]    in_rdata;        // desde mem_in (compartido)

  // Escritura de salida (mem_out) para cada core
  logic [AW-1:0] out_waddr_seq;
  logic [7:0]    out_wdata_seq;
  logic          out_we_seq;

  logic [AW-1:0] out_waddr_simd;
  logic [7:0]    out_wdata_simd;
  logic          out_we_simd;

  // Señales agregadas desde el core activo (antes de clearing)
  logic [AW-1:0] out_waddr_core;
  logic [7:0]    out_wdata_core;
  logic          out_we_core;

  // Señales hacia la BRAM de salida (después de clearing)
  logic [AW-1:0] mem_out_waddr;
  logic [7:0]    mem_out_wdata;
  logic          mem_out_we;

  // Performance counters desde cada core
  logic [31:0] perf_flops_seq;
  logic [31:0] perf_mem_rd_seq;
  logic [31:0] perf_mem_wr_seq;

  logic [31:0] perf_flops_simd;
  logic [31:0] perf_mem_rd_simd;
  logic [31:0] perf_mem_wr_simd;

  // Performance visibles por JTAG (del core activo)
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

  // busy/done agregados para status y bloqueo de escritura por JTAG
  assign busy_any = busy_seq | busy_simd;
  assign done_any = done_seq | done_simd;

  // Selección de performance según modo
  always_comb begin
    if (mode_simd) begin
      perf_flops   = perf_flops_simd;
      perf_mem_rd  = perf_mem_rd_simd;
      perf_mem_wr  = perf_mem_wr_simd;
    end else begin
      perf_flops   = perf_flops_seq;
      perf_mem_rd  = perf_mem_rd_seq;
      perf_mem_wr  = perf_mem_wr_seq;
    end
  end

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
    .mode_simd     (mode_simd_jtag),
    .cfg_in_w      (in_w_cfg),
    .cfg_in_h      (in_h_cfg),
    .cfg_scale_q88 (scale_q88_cfg),
    .status_done   (done_any),
    .status_busy   (busy_any),
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
  // 2b) Sincronizador del switch de modo SIMD + LED
  // =========================================================================
  logic mode_sw_meta, mode_sw_sync;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      mode_sw_meta <= 1'b0;
      mode_sw_sync <= 1'b0;
    end else begin
      mode_sw_meta <= mode_simd_sw;
      mode_sw_sync <= mode_sw_meta;
    end
  end

  // Modo efectivo: OR entre switch físico y bit de JTAG
  assign mode_simd     = mode_sw_sync | mode_simd_jtag;
  assign led_mode_simd = mode_simd;

  // Start por JTAG o por switch
  logic start_any;
  always_comb begin
    start_any = start_pulse_jtag | start_pulse_sw;
  end

  // Separar start para cada núcleo según modo
  logic start_seq, start_simd;
  assign start_seq  = start_any & ~mode_simd;
  assign start_simd = start_any &  mode_simd;

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

  // Selección de dirección de lectura para la BRAM de entrada
  assign in_raddr_core = mode_simd ? in_raddr_simd : in_raddr_seq;

  onchip_mem_dp #(
    .ADDR_W (AW),
    .INIT_EN(1'b1)              // simulación: carga MEM_INIT_FILE (si está definido)
  ) mem_in (
    .clk   (clk_50),
    .raddr (in_raddr_core),
    .rdata (in_rdata),
    .waddr (jtag_in_waddr),
    .wdata (jtag_in_wdata),
    .we    (jtag_in_we & ~busy_any) // evitar escribir durante procesamiento de cualquier core
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
    .we    (jtag_in_we & ~busy_any)
  );

  // Memoria de salida: selección de core activo
  assign out_waddr_core = mode_simd ? out_waddr_simd : out_waddr_seq;
  assign out_wdata_core = mode_simd ? out_wdata_simd : out_wdata_seq;
  assign out_we_core    = mode_simd ? out_we_simd    : out_we_seq;

  // --- Lógica de clearing de mem_out tras reset ---
  logic        clearing;
  logic [AW-1:0] clear_addr;

  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      // Al entrar en reset, se marca que hay que limpiar la memoria de salida
      clearing   <= 1'b1;
      clear_addr <= '0;
    end else begin
      if (clearing) begin
        clear_addr <= clear_addr + 1'b1;
        // Cuando se recorre todo el espacio de direcciones, se termina
        if (clear_addr == {AW{1'b1}}) begin
          clearing <= 1'b0;
        end
      end
    end
  end

  // Multiplexor entre escrituras de clearing y escrituras del core
  always_comb begin
    if (clearing) begin
      mem_out_we    = 1'b1;
      mem_out_waddr = clear_addr;
      mem_out_wdata = 8'h00;
    end else begin
      mem_out_we    = out_we_core;
      mem_out_waddr = out_waddr_core;
      mem_out_wdata = out_wdata_core;
    end
  end

  onchip_mem_dp #(
    .ADDR_W (AW),
    .INIT_EN(1'b0)
  ) mem_out (
    .clk   (clk_50),
    .raddr (jtag_out_raddr),
    .rdata (jtag_out_rdata),
    .waddr (mem_out_waddr),
    .wdata (mem_out_wdata),
    .we    (mem_out_we)
  );

  // =========================================================================
  // 5) Parámetros desde JTAG
  // =========================================================================
  assign in_w      = in_w_cfg;
  assign in_h      = in_h_cfg;
  assign scale_q88 = scale_q88_cfg;

  // =========================================================================
  // 6) Núcleo bilineal secuencial
  // =========================================================================
  bilinear_seq #(.AW(AW)) core_seq (
    .clk           (clk_50),
    .rst_n         (rst_n),
    .start         (start_seq),
    .busy          (busy_seq),
    .done          (done_seq),

    .i_step_en     (1'b0),   // stepping desactivado por ahora
    .i_step_pulse  (1'b0),

    .i_in_w        (in_w),
    .i_in_h        (in_h),
    .i_scale_q88   (scale_q88),

    .o_out_w       (out_w_s),
    .o_out_h       (out_h_s),

    .in_raddr      (in_raddr_seq),
    .in_rdata      (in_rdata),

    .out_waddr     (out_waddr_seq),
    .out_wdata     (out_wdata_seq),
    .out_we        (out_we_seq),

    .o_flop_count  (perf_flops_seq),
    .o_mem_rd_count(perf_mem_rd_seq),
    .o_mem_wr_count(perf_mem_wr_seq)
  );

  // =========================================================================
  // 6b) Núcleo bilineal "SIMD" (mismo interfaz, FLOPs escalados por LANES)
  // =========================================================================
  bilinear_simd #(.AW(AW), .LANES(LANES_SIMD)) core_simd (
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

    .o_out_w       (), // no se usan por ahora
    .o_out_h       (),

    .in_raddr      (in_raddr_simd),
    .in_rdata      (in_rdata),

    .out_waddr     (out_waddr_simd),
    .out_wdata     (out_wdata_simd),
    .out_we        (out_we_simd),

    .o_flop_count  (perf_flops_simd),
    .o_mem_rd_count(perf_mem_rd_simd),
    .o_mem_wr_count(perf_mem_wr_simd)
  );

  // =========================================================================
  // 7) LED done latcheado (cualquier core)
  // =========================================================================
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) led_done <= 1'b0;
    else begin
      if (start_any)    led_done <= 1'b0;
      else if (done_any) led_done <= 1'b1;
    end
  end

endmodule
