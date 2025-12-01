`timescale 1ps/1ps

// ============================================================================
// top_dsa_seq.sv
//   - Cores secuencial y SIMD4
//   - mem_in  (entrada)  : SDRAM externa (lectura por cores y JTAG)
//   - mem_out (salida)   : wide_onchip_mem (BRAM 32 bits, escritura core,
//                          lectura JTAG a bytes)
//   - Control por switch (start_sw / mode_simd_sw) y por JTAG (jtag_connect)
// ============================================================================

`define MEM_INIT_FILE "C:/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_16x16.hex"

module top_dsa_seq #(
  parameter int AW             = 16,   // ancho de dirección de palabra de 32 bits
  parameter bit SIMULATION     = 0,
  parameter int DEB_W          = 20,
  parameter int RST_STRETCH_W  = 22
)(
  input  logic clk_50,
  input  logic rst_n,

  input  logic start_sw,
  input  logic mode_simd_sw,

  output logic led_done,
  output logic led_reset_evt,
  output logic led_start_on,
  output logic led_simd_mode,

  // Puertos físicos de SDRAM externa 64 MB (solo mem_in)
  output logic [12:0] DRAM_ADDR,
  output logic  [1:0] DRAM_BA,
  output logic        DRAM_CAS_N,
  output logic        DRAM_CKE,
  output logic        DRAM_CLK,
  output logic        DRAM_CS_N,
  inout  wire  [15:0] DRAM_DQ,
  output logic  [1:0] DRAM_DQM,
  output logic        DRAM_RAS_N,
  output logic        DRAM_WE_N
);

  // =========================================================================
  // ---------------------- CONFIGURACIÓN (JTAG + switches) -------------------
  // =========================================================================

  // --- Señales desde JTAG ---
  logic        jtag_start_pulse;
  logic [15:0] jtag_cfg_in_w;
  logic [15:0] jtag_cfg_in_h;
  logic [15:0] jtag_cfg_scale_q88;
  logic        jtag_cfg_mode_simd;

  // --- Config efectiva hacia los cores ---
  logic [15:0] in_w;
  logic [15:0] in_h;
  logic [15:0] scale_q88;
  logic        mode_simd_eff;

  // *** IMPORTANTE ***
  // Los cores ven directamente los registros que maneja jtag_connect.
  // El GUI Tcl escribe IN_W / IN_H / SCALE_Q88 y también los lee para
  // el Auto-fill, así que no se filtran ni se reinterpretan aquí.
  assign in_w      = jtag_cfg_in_w;
  assign in_h      = jtag_cfg_in_h;
  assign scale_q88 = jtag_cfg_scale_q88;

  // modo SIMD: OR entre switch físico y registro de JTAG
  assign mode_simd_eff = mode_simd_sw | jtag_cfg_mode_simd;
  assign led_simd_mode = mode_simd_eff;

  // =========================================================================
  // Start switch: simulación vs hardware
  // =========================================================================
  logic               sw_meta, sw_sync;
  logic               sw_debounced, sw_debounced_q;
  logic [DEB_W-1:0]   deb_cnt;
  logic               sw_sync_q;
  wire                start_pulse_sw;

  generate
    if (SIMULATION) begin : GEN_START_SIM
      // En simulación se usa directamente el switch, sin debounce
      assign start_pulse_sw = start_sw;
      assign led_start_on   = start_sw;

      always_ff @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) begin
          sw_meta        <= 1'b0;
          sw_sync        <= 1'b0;
          sw_sync_q      <= 1'b0;
          sw_debounced   <= 1'b0;
          sw_debounced_q <= 1'b0;
          deb_cnt        <= '0;
        end else begin
          sw_meta        <= start_sw;
          sw_sync        <= sw_meta;
          sw_sync_q      <= sw_sync;
          sw_debounced   <= start_sw;
          sw_debounced_q <= sw_debounced;
          deb_cnt        <= '0;
        end
      end
    end else begin : GEN_START_HW
      // Debouncer "real" para FPGA
      always_ff @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) begin
          sw_meta        <= 1'b0;
          sw_sync        <= 1'b0;
          sw_sync_q      <= 1'b0;
          deb_cnt        <= '0;
          sw_debounced   <= 1'b0;
          sw_debounced_q <= 1'b0;
        end else begin
          sw_meta <= start_sw;
          sw_sync <= sw_meta;

          // Detecta un cambio en el nivel sincronizado
          if (sw_sync != sw_sync_q) begin
            sw_sync_q <= sw_sync;
            deb_cnt   <= '0;
          end else if (deb_cnt != {DEB_W{1'b1}}) begin
            deb_cnt   <= deb_cnt + 1'b1;
          end

          // Cuando se mantiene estable lo suficiente, actualiza el valor "debounced"
          if (deb_cnt == {DEB_W{1'b1}}) begin
            sw_debounced <= sw_sync_q;
          end

          sw_debounced_q <= sw_debounced;
        end
      end

      assign start_pulse_sw = sw_debounced & ~sw_debounced_q;
      assign led_start_on   = sw_debounced;
    end
  endgenerate

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
  // Señales de control global start/busy/done
  // =========================================================================
  logic start_any;
  logic busy_seq,  done_seq;
  logic busy_simd, done_simd;
  logic busy,      done;

  // start puede venir de switch o de JTAG
  assign start_any = start_pulse_sw | jtag_start_pulse;

  assign busy = mode_simd_eff ? busy_simd : busy_seq;
  assign done = mode_simd_eff ? done_simd : done_seq;

  // =========================================================================
  // --------------------------- MEMORIA DE ENTRADA ---------------------------
  //    - mem_in: sdram_wide_if @ BASE_ADDR = 0
  //    - Acceso por cores (lectura) y por JTAG (lectura/escritura)
  // =========================================================================
  localparam [24:0] MEM_IN_BASE = 25'd0;

  // Interfaz SDRAM interna para mem_in
  logic              mem_in_req_valid_s;
  logic              mem_in_req_ready_s;
  logic              mem_in_req_we_s;
  logic [AW-1:0]     mem_in_req_addr_s;
  logic [31:0]       mem_in_req_wdata_s;
  logic              mem_in_resp_valid_s;
  logic [31:0]       mem_in_resp_rdata_s;

  // Interfaz lógica hacia cores (lectura 32b)
  logic              in_req_valid_seq;
  logic              in_req_ready_seq;
  logic [AW-1:0]     in_req_addr_seq;
  logic              in_resp_valid_seq;
  logic [31:0]       in_resp_rdata_seq;

  logic              in_req_valid_simd;
  logic              in_req_ready_simd;
  logic [AW-1:0]     in_req_addr_simd;
  logic              in_resp_valid_simd;
  logic [31:0]       in_resp_rdata_simd;

  // Interfaz de JTAG hacia mem_in (32b handshake)
  logic              jtag_mem_req_valid;
  logic              jtag_mem_req_we;
  logic [AW-1:0]     jtag_mem_req_addr;
  logic [31:0]       jtag_mem_req_wdata;
  logic              jtag_mem_req_ready;
  logic              jtag_mem_resp_valid;
  logic [31:0]       jtag_mem_resp_rdata;

  // Multiplexor de acceso a mem_in: cuando busy=1 usan los cores, cuando busy=0 JTAG
  logic          core_req_valid;
  logic [AW-1:0] core_req_addr;

  assign core_req_valid = mode_simd_eff ? in_req_valid_simd : in_req_valid_seq;
  assign core_req_addr  = mode_simd_eff ? in_req_addr_simd  : in_req_addr_seq;

  always_comb begin
    // Defaults hacia SDRAM
    mem_in_req_valid_s = 1'b0;
    mem_in_req_we_s    = 1'b0;
    mem_in_req_addr_s  = '0;
    mem_in_req_wdata_s = 32'd0;

    // Defaults hacia cores y JTAG
    in_req_ready_seq    = 1'b0;
    in_resp_valid_seq   = 1'b0;
    in_resp_rdata_seq   = 32'd0;

    in_req_ready_simd   = 1'b0;
    in_resp_valid_simd  = 1'b0;
    in_resp_rdata_simd  = 32'd0;

    jtag_mem_req_ready  = 1'b0;
    jtag_mem_resp_valid = 1'b0;
    jtag_mem_resp_rdata = 32'd0;

    if (busy) begin
      // ------------------- Cuando el core está corriendo -------------------
      mem_in_req_valid_s = core_req_valid;
      mem_in_req_we_s    = 1'b0;    // sólo lectura desde cores
      mem_in_req_addr_s  = core_req_addr;

      if (mode_simd_eff) begin
        in_req_ready_simd   = mem_in_req_ready_s;
        in_resp_valid_simd  = mem_in_resp_valid_s;
        in_resp_rdata_simd  = mem_in_resp_rdata_s;
      end else begin
        in_req_ready_seq    = mem_in_req_ready_s;
        in_resp_valid_seq   = mem_in_resp_valid_s;
        in_resp_rdata_seq   = mem_in_resp_rdata_s;
      end
    end else begin
      // ---------------------- Core idle: JTAG manda -----------------------
      mem_in_req_valid_s = jtag_mem_req_valid;
      mem_in_req_we_s    = jtag_mem_req_we;
      mem_in_req_addr_s  = jtag_mem_req_addr;
      mem_in_req_wdata_s = jtag_mem_req_wdata;

      jtag_mem_req_ready  = mem_in_req_ready_s;
      jtag_mem_resp_valid = mem_in_resp_valid_s;
      jtag_mem_resp_rdata = mem_in_resp_rdata_s;
    end
  end

  // Instancia SDRAM wide para mem_in
  sdram_wide_if #(
    .AW       (AW),
    .BASE_ADDR(MEM_IN_BASE)
  ) mem_in (
    .clk        (clk_50),
    .rst_n      (rst_n),

    .req_valid  (mem_in_req_valid_s),
    .req_ready  (mem_in_req_ready_s),
    .req_we     (mem_in_req_we_s),
    .req_addr   (mem_in_req_addr_s),
    .req_wdata  (mem_in_req_wdata_s),
    .resp_valid (mem_in_resp_valid_s),
    .resp_rdata (mem_in_resp_rdata_s),

    .DRAM_ADDR  (DRAM_ADDR),
    .DRAM_BA    (DRAM_BA),
    .DRAM_CAS_N (DRAM_CAS_N),
    .DRAM_CKE   (DRAM_CKE),
    .DRAM_CLK   (DRAM_CLK),
    .DRAM_CS_N  (DRAM_CS_N),
    .DRAM_DQ    (DRAM_DQ),
    .DRAM_DQM   (DRAM_DQM),
    .DRAM_RAS_N (DRAM_RAS_N),
    .DRAM_WE_N  (DRAM_WE_N)
  );

  // =========================================================================
  // --------------------------- MEMORIA DE SALIDA ---------------------------
  //    - mem_out: wide_onchip_mem (BRAM 32 bits)
  //    - Escrituras 32b desde cores (via mem_out_bridge_wide)
  //    - Lecturas 8b vía JTAG (también en mem_out_bridge_wide)
  // =========================================================================

  // Interfaz interna para mem_out (misma forma que sdram_wide_if)
  logic              mem_out_req_valid_s;
  logic              mem_out_req_ready_s;
  logic              mem_out_req_we_s;
  logic [AW-1:0]     mem_out_req_addr_s;
  logic [31:0]       mem_out_req_wdata_s;
  logic              mem_out_resp_valid_s;
  logic [31:0]       mem_out_resp_rdata_s;

  // Memoria de salida on-chip (BRAM wide 32 bits)
  wide_onchip_mem #(
    .ADDR_W         (AW),
    .CLEAR_ON_RESET (1'b1)
  ) mem_out (
    .clk        (clk_50),
    .rst_n      (rst_n),

    .req_valid  (mem_out_req_valid_s),
    .req_ready  (mem_out_req_ready_s),
    .req_we     (mem_out_req_we_s),
    .req_addr   (mem_out_req_addr_s),
    .req_wdata  (mem_out_req_wdata_s),

    .resp_valid (mem_out_resp_valid_s),
    .resp_rdata (mem_out_resp_rdata_s)
  );



  // -------------------------------------------------------------------------
  // Salidas de los cores (mem_out lógica)
  // -------------------------------------------------------------------------
  logic [AW-1:0] out_mem_addr_seq;
  logic [31:0]   out_mem_wdata_seq;
  logic          out_mem_we_seq;

  logic [AW-1:0] out_mem_addr_simd;
  logic [31:0]   out_mem_wdata_simd;
  logic          out_mem_we_simd;

  // -------------------------------------------------------------------------
  // Puente hacia mem_out: FIFO + writer + reader + arbitraje (modularizado)
  // -------------------------------------------------------------------------
  logic [AW-1:0] out_mem_raddr;  // desde jtag_connect (índice de pixel)
  logic  [7:0]   out_mem_rdata;  // byte hacia jtag_connect

  mem_out_bridge_wide #(
    .AW            (AW),
    .OUT_FIFO_DEPTH(32)
  ) u_mem_out_bridge (
    .clk                (clk_50),
    .rst_n              (rst_n),
    .mode_simd_eff      (mode_simd_eff),

    .out_mem_addr_seq   (out_mem_addr_seq),
    .out_mem_wdata_seq  (out_mem_wdata_seq),
    .out_mem_we_seq     (out_mem_we_seq),

    .out_mem_addr_simd  (out_mem_addr_simd),
    .out_mem_wdata_simd (out_mem_wdata_simd),
    .out_mem_we_simd    (out_mem_we_simd),

    .mem_out_req_valid  (mem_out_req_valid_s),
    .mem_out_req_ready  (mem_out_req_ready_s),
    .mem_out_req_we     (mem_out_req_we_s),
    .mem_out_req_addr   (mem_out_req_addr_s),
    .mem_out_req_wdata  (mem_out_req_wdata_s),
    .mem_out_resp_valid (mem_out_resp_valid_s),
    .mem_out_resp_rdata (mem_out_resp_rdata_s),

    .jtag_out_mem_raddr (out_mem_raddr),
    .jtag_out_mem_rdata (out_mem_rdata)
  );

  // =========================================================================
  // ----------------------------- INSTANCIA CORES ---------------------------
  // =========================================================================

  // Dimensiones de salida
  logic [15:0] out_w_seq,  out_h_seq;
  logic [15:0] out_w_simd, out_h_simd;

  // Contadores de desempeño
  logic [31:0] flops_seq, mem_rd_seq, mem_wr_seq;
  logic [31:0] flops_simd, mem_rd_simd, mem_wr_simd;

  logic start_seq, start_simd;
  assign start_seq  = start_any & ~mode_simd_eff;
  assign start_simd = start_any &  mode_simd_eff;

  // ---------------- Core secuencial ----------------
  bilinear_seq_wide #(
    .AW (AW)
  ) u_core_seq (
    .clk          (clk_50),
    .rst_n        (rst_n),
    .start        (start_seq),
    .busy         (busy_seq),
    .done         (done_seq),

    .i_step_en    (1'b0),
    .i_step_pulse (1'b0),

    .i_in_w       (in_w),
    .i_in_h       (in_h),
    .i_scale_q88  (scale_q88),

    .o_out_w      (out_w_seq),
    .o_out_h      (out_h_seq),

    .in_req_valid (in_req_valid_seq),
    .in_req_ready (in_req_ready_seq),
    .in_req_addr  (in_req_addr_seq),
    .in_resp_valid(in_resp_valid_seq),
    .in_resp_rdata(in_resp_rdata_seq),

    .out_mem_addr  (out_mem_addr_seq),
    .out_mem_wdata (out_mem_wdata_seq),
    .out_mem_we    (out_mem_we_seq),

    .o_flop_count   (flops_seq),
    .o_mem_rd_count (mem_rd_seq),
    .o_mem_wr_count (mem_wr_seq)
  );

  // ---------------- Core SIMD x4 ----------------
  bilinear_simd4_wide #(
    .AW (AW)
  ) u_core_simd4 (
    .clk          (clk_50),
    .rst_n        (rst_n),
    .start        (start_simd),
    .busy         (busy_simd),
    .done         (done_simd),

    .i_step_en    (1'b0),
    .i_step_pulse (1'b0),

    .i_in_w       (in_w),
    .i_in_h       (in_h),
    .i_scale_q88  (scale_q88),

    .o_out_w      (out_w_simd),
    .o_out_h      (out_h_simd),

    .in_req_valid (in_req_valid_simd),
    .in_req_ready (in_req_ready_simd),
    .in_req_addr  (in_req_addr_simd),
    .in_resp_valid(in_resp_valid_simd),
    .in_resp_rdata(in_resp_rdata_simd),

    .out_mem_addr  (out_mem_addr_simd),
    .out_mem_wdata (out_mem_wdata_simd),
    .out_mem_we    (out_mem_we_simd),

    .o_flop_count   (flops_simd),
    .o_mem_rd_count (mem_rd_simd),
    .o_mem_wr_count (mem_wr_simd)
  );

  // =========================================================================
  // ---------------------------- LED DONE (flanco) --------------------------
  // =========================================================================
  logic done_seq_q, done_simd_q;

  wire done_seq_pulse  = done_seq  & ~done_seq_q;
  wire done_simd_pulse = done_simd & ~done_simd_q;
  wire done_pulse      = mode_simd_eff ? done_simd_pulse : done_seq_pulse;

  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      done_seq_q  <= 1'b0;
      done_simd_q <= 1'b0;
    end else begin
      done_seq_q  <= done_seq;
      done_simd_q <= done_simd;
    end
  end

  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      led_done <= 1'b0;
    end else begin
      if (start_any) begin
        led_done <= 1'b0;
      end else if (done_pulse) begin
        led_done <= 1'b1;
      end
    end
  end

  // =========================================================================
  // --------------------------- CONEXIÓN JTAG -------------------------------
  // =========================================================================

  // Señales del Virtual JTAG
  logic        vj_tck;
  logic        vj_tdi;
  logic        vj_tdo;
  logic [1:0]  vj_ir_in;
  logic        vj_cdr;
  logic        vj_sdr;
  logic        vj_udr;

  // IP de Virtual JTAG
  sld_virtual_jtag #(
    .sld_auto_instance_index ("YES"),
    .sld_instance_index      (0),
    .sld_ir_width            (2)
  ) virtual_jtag_0 (
    .tdi                (vj_tdi),
    .tdo                (vj_tdo),
    .ir_in              (vj_ir_in),
    .ir_out             (),           // no usado
    .virtual_state_cdr  (vj_cdr),
    .virtual_state_sdr  (vj_sdr),
    .virtual_state_e1dr (),
    .virtual_state_pdr  (),
    .virtual_state_e2dr (),
    .virtual_state_udr  (vj_udr),
    .virtual_state_cir  (),
    .virtual_state_uir  (),
    .tck                (vj_tck)
  );

  // -------------------------------------------------------------------------
  // jtag_connect: interfaz de registros + acceso a mem_in/mem_out
  // -------------------------------------------------------------------------
  // Señales de performance y dims activas hacia JTAG (según modo)
  logic [31:0] perf_flops_mux, perf_mem_rd_mux, perf_mem_wr_mux;
  logic [15:0] out_w_active, out_h_active;

  assign perf_flops_mux   = mode_simd_eff ? flops_simd   : flops_seq;
  assign perf_mem_rd_mux  = mode_simd_eff ? mem_rd_simd  : mem_rd_seq;
  assign perf_mem_wr_mux  = mode_simd_eff ? mem_wr_simd  : mem_wr_seq;
  assign out_w_active     = mode_simd_eff ? out_w_simd   : out_w_seq;
  assign out_h_active     = mode_simd_eff ? out_h_simd   : out_h_seq;

  jtag_connect #(
    .DRW (40),
    .AW  (AW)
  ) u_jtag_connect (
    .tck           (vj_tck),
    .tdi           (vj_tdi),
    .tdo           (vj_tdo),
    .ir_in         (vj_ir_in),

    .vs_cdr        (vj_cdr),
    .vs_sdr        (vj_sdr),
    .vs_udr        (vj_udr),

    .start_pulse   (jtag_start_pulse),
    .cfg_in_w      (jtag_cfg_in_w),
    .cfg_in_h      (jtag_cfg_in_h),
    .cfg_scale_q88 (jtag_cfg_scale_q88),
    .cfg_mode_simd (jtag_cfg_mode_simd),

    .status_done   (done),
    .status_busy   (busy),
    .perf_flops    (perf_flops_mux),
    .perf_mem_rd   (perf_mem_rd_mux),
    .perf_mem_wr   (perf_mem_wr_mux),

    .out_w         (out_w_active),
    .out_h         (out_h_active),

    // Acceso a memoria de entrada (mem_in)
    .mem_req_valid (jtag_mem_req_valid),
    .mem_req_we    (jtag_mem_req_we),
    .mem_req_addr  (jtag_mem_req_addr),
    .mem_req_wdata (jtag_mem_req_wdata),
    .mem_req_ready (jtag_mem_req_ready),
    .mem_resp_valid(jtag_mem_resp_valid),
    .mem_resp_rdata(jtag_mem_resp_rdata),

    // Vista 8 bits de mem_out (leída vía puente BRAM wide)
    .out_mem_raddr (out_mem_raddr),
    .out_mem_rdata (out_mem_rdata),

    .clk_sys       (clk_50),
    .rst_sys_n     (rst_n)
  );

endmodule
