// ============================================================================
// top_dsa_seq.sv — Top con Virtual JTAG, LEDs y BRAMs (lectura/escritura)
// Núcleo secuencial + núcleo SIMD4, seleccionables por modo (JTAG + switch).
// Añadido: limpieza de mem_out tras reset.
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

  // Señales de estado global (después de multiplexar núcleos)
  logic        busy;   // desde núcleo seleccionado
  logic        done;   // desde núcleo seleccionado

  // Configuración desde JTAG
  logic [15:0] in_w_cfg, in_h_cfg, scale_q88_cfg;
  logic [15:0] in_w, in_h, scale_q88;

  // Modo SIMD desde JTAG y efectivo
  logic        mode_simd_cfg;
  logic        mode_simd_eff;   // OR de JTAG + switch físico

  // Señales de lectura/escritura de BRAM compartidas
  logic [AW-1:0] in_raddr_core;
  logic [7:0]    in_rdata;

  logic [AW-1:0] out_waddr_core;
  logic [7:0]    out_wdata_core;
  logic          out_we_core;

  // Señales hacia mem_out, tras multiplexar con limpieza
  logic [AW-1:0] out_waddr_mux;
  logic [7:0]    out_wdata_mux;
  logic          out_we_mux;

  // Performance counters globales (núcleo seleccionado)
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

    // Modo SIMD desde JTAG
    .cfg_mode_simd (mode_simd_cfg),

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
  // Lectura desde núcleo seleccionado / escritura desde JTAG (upload)
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

  // BRAM de entrada para vista por JTAG (lectura)
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

  // BRAM de salida (escritura desde núcleo o desde limpiador, lectura por JTAG)
  onchip_mem_dp #(
    .ADDR_W (AW),
    .INIT_EN(1'b0)
  ) mem_out (
    .clk   (clk_50),
    .raddr (jtag_out_raddr),
    .rdata (jtag_out_rdata),
    .waddr (out_waddr_mux),
    .wdata (out_wdata_mux),
    .we    (out_we_mux)
  );

  // =========================================================================
  // 5) Parámetros desde JTAG
  // =========================================================================
  assign in_w      = in_w_cfg;
  assign in_h      = in_h_cfg;
  assign scale_q88 = scale_q88_cfg;

  // Modo SIMD efectivo: OR entre JTAG y switch físico
  assign mode_simd_eff = mode_simd_cfg | mode_simd_sw;
  assign led_simd_mode = mode_simd_eff;

  // =========================================================================
  // 6) Limpieza de mem_out tras reset
  // =========================================================================
  localparam int OUT_DEPTH = (1 << AW);

  logic                clear_active;
  logic [AW-1:0]       clear_addr;

  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      clear_active <= 1'b1;          // tras cualquier reset, limpiar mem_out
      clear_addr   <= {AW{1'b0}};
    end else begin
      if (clear_active) begin
        clear_addr <= clear_addr + 1'b1;
        if (clear_addr == OUT_DEPTH-1)
          clear_active <= 1'b0;
      end
    end
  end

  // Mux de escritura a mem_out: limpiador vs núcleo
  assign out_waddr_mux = clear_active ? clear_addr       : out_waddr_core;
  assign out_wdata_mux = clear_active ? 8'h00            : out_wdata_core;
  assign out_we_mux    = clear_active ? 1'b1             : out_we_core;

  // =========================================================================
  // 7) Start global (bloqueado mientras se limpia mem_out)
  // =========================================================================
  logic start_any_raw;
  logic start_any;

  always_comb begin
    start_any_raw = start_pulse_jtag | start_pulse_sw;
    start_any     = start_any_raw & ~clear_active;  // no iniciar mientras se limpia mem_out
  end

  // =========================================================================
  // 8) Núcleos bilineales: secuencial + SIMD4
  // =========================================================================

  // Señales núcleo secuencial
  logic        busy_seq,  done_seq;
  logic [AW-1:0] in_raddr_seq;
  logic [AW-1:0] out_waddr_seq;
  logic [7:0]    out_wdata_seq;
  logic          out_we_seq;
  logic [31:0]   perf_flops_seq, perf_mem_rd_seq, perf_mem_wr_seq;
  logic [15:0]   out_w_s_seq, out_h_s_seq;

  // Señales núcleo SIMD4
  logic        busy_simd, done_simd;
  logic [AW-1:0] in_raddr_simd;
  logic [AW-1:0] out_waddr_simd;
  logic [7:0]    out_wdata_simd;
  logic          out_we_simd;
  logic [31:0]   perf_flops_simd, perf_mem_rd_simd, perf_mem_wr_simd;
  logic [15:0]   out_w_s_simd, out_h_s_simd;

  // Señales de start hacia cada núcleo (solo uno activo)
  logic start_seq, start_simd;
  assign start_seq  = start_any & ~mode_simd_eff;
  assign start_simd = start_any &  mode_simd_eff;

  // Multiplexor de direcciones/datos de memoria hacia BRAM
  assign in_raddr_core  = mode_simd_eff ? in_raddr_simd  : in_raddr_seq;
  assign out_waddr_core = mode_simd_eff ? out_waddr_simd : out_waddr_seq;
  assign out_wdata_core = mode_simd_eff ? out_wdata_simd : out_wdata_seq;
  assign out_we_core    = mode_simd_eff ? out_we_simd    : out_we_seq;

  // Multiplexor de estado/performance hacia JTAG
  assign busy         = mode_simd_eff ? busy_simd        : busy_seq;
  assign done         = mode_simd_eff ? done_simd        : done_seq;
  assign perf_flops   = mode_simd_eff ? perf_flops_simd  : perf_flops_seq;
  assign perf_mem_rd  = mode_simd_eff ? perf_mem_rd_simd : perf_mem_rd_seq;
  assign perf_mem_wr  = mode_simd_eff ? perf_mem_wr_simd : perf_mem_wr_seq;

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
    .in_rdata      (in_rdata),

    .out_waddr     (out_waddr_seq),
    .out_wdata     (out_wdata_seq),
    .out_we        (out_we_seq),

    .o_flop_count  (perf_flops_seq),
    .o_mem_rd_count(perf_mem_rd_seq),
    .o_mem_wr_count(perf_mem_wr_seq)
  );

  // Núcleo SIMD4
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
  // 9) LED done latcheado (independiente del modo)
  // =========================================================================
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) led_done <= 1'b0;
    else begin
      if (start_any) led_done <= 1'b0;
      else if (done) led_done <= 1'b1;
    end
  end

endmodule
