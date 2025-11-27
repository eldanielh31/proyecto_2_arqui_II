// ============================================================================
// top_dsa_wide.sv
//   - Top module with wide memory architecture
//   - Single input memory (32-bit wide words)
//   - 2 read ports to match M10K constraints
//   - Sequential and SIMD4 cores share same wide memory
//   - Adapted to Virtual JTAG (byte view) via jtag_connect
// ============================================================================

`define MEM_INIT_FILE "C:/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_16x16.hex"

module top_dsa_seq #(

  // Tamaño de imagen parametrizable
  parameter int IMG_W = 16,
  parameter int IMG_H = 16,

  parameter int AW = 10,      // Byte address width (reduced if using smaller images)
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

  // =========================================================================
  // Core / status / config signals
  // =========================================================================
  logic start_pulse_sw;
  logic busy, done;

  logic [15:0] in_w;
  logic [15:0] in_h;
  logic [15:0] scale_q88;

  logic        mode_simd_eff;

  // Performance counters (selected between seq / simd for JTAG)
  logic [31:0] perf_flops,   perf_mem_rd,   perf_mem_wr;
  logic [31:0] perf_flops_seq, perf_mem_rd_seq, perf_mem_wr_seq;
  logic [31:0] perf_flops_simd, perf_mem_rd_simd, perf_mem_wr_simd;

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
  //   - mem_in_raddr*_core: direcciones usadas por los cores
  //   - mem_in_raddr*: direcciones físicas hacia wide_onchip_mem (mux core/JTAG)
  logic [AW-1:0] mem_in_raddr0_core, mem_in_raddr1_core;
  logic [AW-1:0] mem_in_raddr0,      mem_in_raddr1;
  logic [31:0]   mem_in_rdata0,      mem_in_rdata1;

  // Escritura a mem_in desde JTAG (byte view)
  logic [AW-1:0] mem_in_waddr0;
  logic  [7:0]   mem_in_wdata0;
  logic          mem_in_we0;

  wide_onchip_mem #(
    .ADDR_W(AW),
    .INIT_EN(1'b1)
  ) mem_in (
    .clk   (clk_50),
    .raddr0(mem_in_raddr0),
    .rdata0(mem_in_rdata0),
    .raddr1(mem_in_raddr1),
    .rdata1(mem_in_rdata1),
    .waddr0(mem_in_waddr0),
    .wdata0(mem_in_wdata0),
    .we0   (mem_in_we0),
    .waddr1('0), .wdata1(8'h00), .we1(1'b0),
    .waddr2('0), .wdata2(8'h00), .we2(1'b0),
    .waddr3('0), .wdata3(8'h00), .we3(1'b0)
  );

  // Output memory (wide, 4 write ports)
  //   - mem_out_raddr_jtag: lectura solo desde JTAG
  logic [AW-1:0] mem_out_waddr0, mem_out_waddr1, mem_out_waddr2, mem_out_waddr3;
  logic  [7:0]   mem_out_wdata0, mem_out_wdata1, mem_out_wdata2, mem_out_wdata3;
  logic          mem_out_we0,    mem_out_we1,    mem_out_we2,    mem_out_we3;

  logic [AW-1:0] mem_out_raddr_jtag;
  logic [31:0]   mem_out_rdata_jtag;

  wide_onchip_mem #(
    .ADDR_W(AW),
    .INIT_EN(1'b0)
  ) mem_out (
    .clk   (clk_50),
    .raddr0(mem_out_raddr_jtag), // sólo JTAG
    .rdata0(mem_out_rdata_jtag),
    .raddr1('0), .rdata1(),
    .waddr0(mem_out_waddr0),
    .wdata0(mem_out_wdata0),
    .we0   (mem_out_we0),
    .waddr1(mem_out_waddr1),
    .wdata1(mem_out_wdata1),
    .we1   (mem_out_we1),
    .waddr2(mem_out_waddr2),
    .wdata2(mem_out_wdata2),
    .we2   (mem_out_we2),
    .waddr3(mem_out_waddr3),
    .wdata3(mem_out_wdata3),
    .we3   (mem_out_we3)
  );

  // =========================================================================
  // JTAG bridge: byte view -> wide memories
  // =========================================================================

  // Byte-address width for JTAG (coincide con AW)
  localparam int AW_JTAG  = AW;
  localparam int AW_WORD  = (AW_JTAG > 2) ? (AW_JTAG - 2) : 1; // índice de palabra

  // Señales de config y control desde JTAG
  logic        jtag_start_pulse;
  logic [15:0] jtag_cfg_in_w;
  logic [15:0] jtag_cfg_in_h;
  logic [15:0] jtag_cfg_scale_q88;
  logic        jtag_cfg_mode_simd;

  // Vista de 8 bits / byte-lineal para JTAG
  logic [AW_JTAG-1:0] jtag_in_mem_raddr;
  logic  [7:0]        jtag_in_mem_rdata;
  logic [AW_JTAG-1:0] jtag_in_mem_waddr;
  logic  [7:0]        jtag_in_mem_wdata;
  logic               jtag_in_mem_we;

  logic [AW_JTAG-1:0] jtag_out_mem_raddr;
  logic  [7:0]        jtag_out_mem_rdata;

  // Conversión dirección byte -> índice de palabra + byte_sel
  logic [AW_WORD-1:0] in_word_addr_jtag;
  logic [1:0]         in_byte_sel_jtag;
  logic [AW_WORD-1:0] out_word_addr_jtag;
  logic [1:0]         out_byte_sel_jtag;

  assign in_word_addr_jtag  = jtag_in_mem_raddr [AW_JTAG-1:2]; // >>2
  assign in_byte_sel_jtag   = jtag_in_mem_raddr [1:0];
  assign out_word_addr_jtag = jtag_out_mem_raddr[AW_JTAG-1:2];
  assign out_byte_sel_jtag  = jtag_out_mem_raddr[1:0];

  // Lectura mem_in: arbitraje core vs JTAG
  //   - mientras el core está ocupado (busy=1), sólo el core usa mem_in
  logic use_jtag_mem_in;
  assign use_jtag_mem_in = ~busy;

  always_comb begin
    if (use_jtag_mem_in) begin
      // JTAG lee por el puerto 0 (word index; se extiende con ceros en MSB)
      mem_in_raddr0 = {2'b00, in_word_addr_jtag};
      // El puerto 1 queda para el core (si se usa) o en 0
      mem_in_raddr1 = mem_in_raddr1_core;
    end else begin
      // Sólo el core accede a mem_in
      mem_in_raddr0 = mem_in_raddr0_core;
      mem_in_raddr1 = mem_in_raddr1_core;
    end
  end

  // Selección del byte para JTAG desde mem_in_rdata0
  always_comb begin
    case (in_byte_sel_jtag)
      2'b00: jtag_in_mem_rdata = mem_in_rdata0[7:0];
      2'b01: jtag_in_mem_rdata = mem_in_rdata0[15:8];
      2'b10: jtag_in_mem_rdata = mem_in_rdata0[23:16];
      2'b11: jtag_in_mem_rdata = mem_in_rdata0[31:24];
    endcase
  end

  // Escritura mem_in desde JTAG:
  //   - jtag_in_mem_waddr es dirección en bytes
  //   - wide_onchip_mem ya interpreta waddr[1:0] como byte_sel y [AW-1:2] como índice de palabra
  always_comb begin
    mem_in_waddr0 = jtag_in_mem_waddr;
    mem_in_wdata0 = jtag_in_mem_wdata;
    mem_in_we0    = jtag_in_mem_we;
  end

  // Lectura mem_out sólo desde JTAG (el core sólo escribe)
  always_comb begin
    mem_out_raddr_jtag = {2'b00, out_word_addr_jtag}; // índice de palabra
  end

  always_comb begin
    case (out_byte_sel_jtag)
      2'b00: jtag_out_mem_rdata = mem_out_rdata_jtag[7:0];
      2'b01: jtag_out_mem_rdata = mem_out_rdata_jtag[15:8];
      2'b10: jtag_out_mem_rdata = mem_out_rdata_jtag[23:16];
      2'b11: jtag_out_mem_rdata = mem_out_rdata_jtag[31:24];
    endcase
  end

  // =========================================================================
  // Virtual JTAG IP + jtag_connect
  // =========================================================================

  // Señales del IP sld_virtual_jtag
  logic       vj_tck;
  logic       vj_tdi;
  logic       vj_tdo;
  logic [1:0] vj_ir_in;
  logic       vj_cdr, vj_sdr, vj_udr;

  // Instancia IP Intel Virtual JTAG (ajustar parámetros si es necesario)
  // Si ya tiene una instancia en otro archivo, puede eliminar esta y conectar
  // vj_tck / vj_tdi / vj_tdo / vj_ir_in / vj_cdr / vj_sdr / vj_udr desde allí.
  sld_virtual_jtag #(
    .sld_auto_instance_index("YES"),
    .sld_instance_index      (0),
    .sld_ir_width            (2),
    .sld_sim_action          (""),
    .sld_sim_n_scan          (0),
    .sld_sim_total_length    (0)
  ) u_vjtag (
    .tck                   (vj_tck),
    .tdi                   (vj_tdi),
    .tdo                   (vj_tdo),
    .ir_in                 (vj_ir_in),
    .ir_out                (/* no usado */),
    .virtual_state_cdr     (vj_cdr),
    .virtual_state_sdr     (vj_sdr),
    .virtual_state_udr     (vj_udr),
    .virtual_state_e1dr    (),
    .virtual_state_e2dr    (),
    .virtual_state_pdr     (),
    .virtual_state_sir     (),
    .virtual_state_tlr     (),
    .virtual_state_rti     (),
    .virtual_state_uir     ()
  );

  // Puente de registros / memoria controlado por JTAG
  jtag_connect #(
    .DRW(40),
    .AW (AW_JTAG)   // ancho de direccionamiento en bytes visto desde el GUI
  ) u_jtag_connect (
    .tck            (vj_tck),
    .tdi            (vj_tdi),
    .tdo            (vj_tdo),
    .ir_in          (vj_ir_in),

    .vs_cdr         (vj_cdr),
    .vs_sdr         (vj_sdr),
    .vs_udr         (vj_udr),

    .start_pulse    (jtag_start_pulse),
    .cfg_in_w       (jtag_cfg_in_w),
    .cfg_in_h       (jtag_cfg_in_h),
    .cfg_scale_q88  (jtag_cfg_scale_q88),
    .cfg_mode_simd  (jtag_cfg_mode_simd),

    .status_done    (done),
    .status_busy    (busy),
    .perf_flops     (perf_flops),
    .perf_mem_rd    (perf_mem_rd),
    .perf_mem_wr    (perf_mem_wr),

    .in_mem_raddr   (jtag_in_mem_raddr),
    .in_mem_rdata   (jtag_in_mem_rdata),

    .in_mem_waddr   (jtag_in_mem_waddr),
    .in_mem_wdata   (jtag_in_mem_wdata),
    .in_mem_we      (jtag_in_mem_we),

    .out_mem_raddr  (jtag_out_mem_raddr),
    .out_mem_rdata  (jtag_out_mem_rdata),

    .clk_sys        (clk_50),
    .rst_sys_n      (rst_n)
  );

  // =========================================================================
  // Configuration (from JTAG + switches)
  // =========================================================================

  // Parámetros de imagen desde registros JTAG
  assign in_w      = jtag_cfg_in_w;
  assign in_h      = jtag_cfg_in_h;
  assign scale_q88 = jtag_cfg_scale_q88;

  // Modo SIMD: OR entre modo configurado por JTAG y switch físico
  assign mode_simd_eff = jtag_cfg_mode_simd | mode_simd_sw;
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
      clear_addr   <= {AW{1'b0}};
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

  // In HW: start desde botón (debounce) y desde JTAG
  assign start_raw = (SIMULATION)
                     ? start_sw
                     : (start_pulse_sw | jtag_start_pulse);

  assign start_any = start_raw & ~clear_active;

  // =========================================================================
  // Core instances
  // =========================================================================

  // Sequential core signals
  logic        busy_seq, done_seq;
  logic [AW-1:0] in_raddr0_seq, in_raddr1_seq;
  logic [31:0]   in_rdata0_seq, in_rdata1_seq;
  logic [AW-1:0] out_waddr_seq;
  logic  [7:0]   out_wdata_seq;
  logic          out_we_seq;

  // SIMD4 core signals
  logic        busy_simd, done_simd;
  logic [AW-1:0] in_raddr0_simd, in_raddr1_simd;
  logic [31:0]   in_rdata0_simd, in_rdata1_simd;
  logic [AW-1:0] out_waddr_simd0, out_waddr_simd1, out_waddr_simd2, out_waddr_simd3;
  logic  [7:0]   out_wdata_simd0, out_wdata_simd1, out_wdata_simd2, out_wdata_simd3;
  logic          out_we_simd0,    out_we_simd1,    out_we_simd2,    out_we_simd3;

  logic start_seq, start_simd;
  assign start_seq  = start_any & ~mode_simd_eff;
  assign start_simd = start_any &  mode_simd_eff;

  // Sequential core
  bilinear_seq_wide #(
    .AW    (AW),
    .IMG_W (IMG_W),
    .IMG_H (IMG_H)
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
    .o_out_w      (),
    .o_out_h      (),
    .in_raddr0    (in_raddr0_seq),
    .in_rdata0    (in_rdata0_seq),
    .in_raddr1    (in_raddr1_seq),
    .in_rdata1    (in_rdata1_seq),
    .out_waddr    (out_waddr_seq),
    .out_wdata    (out_wdata_seq),
    .out_we       (out_we_seq),
    .o_flop_count (perf_flops_seq),
    .o_mem_rd_count(perf_mem_rd_seq),
    .o_mem_wr_count(perf_mem_wr_seq)
  );

  // SIMD4 core
  bilinear_simd4_wide #(
    .AW    (AW),
    .IMG_W (IMG_W),
    .IMG_H (IMG_H)
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
    .o_out_w      (),
    .o_out_h      (),
    .in_raddr0    (in_raddr0_simd),
    .in_rdata0    (in_rdata0_simd),
    .in_raddr1    (in_raddr1_simd),
    .in_rdata1    (in_rdata1_simd),
    .out_waddr0   (out_waddr_simd0),
    .out_wdata0   (out_wdata_simd0),
    .out_we0      (out_we_simd0),
    .out_waddr1   (out_waddr_simd1),
    .out_wdata1   (out_wdata_simd1),
    .out_we1      (out_we_simd1),
    .out_waddr2   (out_waddr_simd2),
    .out_wdata2   (out_wdata_simd2),
    .out_we2      (out_we_simd2),
    .out_waddr3   (out_waddr_simd3),
    .out_wdata3   (out_wdata_simd3),
    .out_we3      (out_we_simd3),
    .o_flop_count (perf_flops_simd),
    .o_mem_rd_count(perf_mem_rd_simd),
    .o_mem_wr_count(perf_mem_wr_simd)
  );

  // =========================================================================
  // Memory read mux (core side)
  // =========================================================================
  always_comb begin
    if (mode_simd_eff) begin
      mem_in_raddr0_core = in_raddr0_simd;
      mem_in_raddr1_core = in_raddr1_simd;
      in_rdata0_simd     = mem_in_rdata0;
      in_rdata1_simd     = mem_in_rdata1;
      in_rdata0_seq      = 32'h0;
      in_rdata1_seq      = 32'h0;
    end else begin
      mem_in_raddr0_core = in_raddr0_seq;
      mem_in_raddr1_core = in_raddr1_seq;
      in_rdata0_seq      = mem_in_rdata0;
      in_rdata1_seq      = mem_in_rdata1;
      in_rdata0_simd     = 32'h0;
      in_rdata1_simd     = 32'h0;
    end
  end

  // =========================================================================
  // Memory write mux (core / clear) hacia mem_out
  // =========================================================================
  always_comb begin
    mem_out_waddr0 = '0; mem_out_wdata0 = 8'h00; mem_out_we0 = 1'b0;
    mem_out_waddr1 = '0; mem_out_wdata1 = 8'h00; mem_out_we1 = 1'b0;
    mem_out_waddr2 = '0; mem_out_wdata2 = 8'h00; mem_out_we2 = 1'b0;
    mem_out_waddr3 = '0; mem_out_wdata3 = 8'h00; mem_out_we3 = 1'b0;

    if (clear_active) begin
      mem_out_waddr0 = clear_addr;
      mem_out_wdata0 = 8'h00;
      mem_out_we0    = 1'b1;
    end else if (mode_simd_eff) begin
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
      mem_out_waddr0 = out_waddr_seq;
      mem_out_wdata0 = out_wdata_seq;
      mem_out_we0    = out_we_seq;
    end
  end

  // =========================================================================
  // Status outputs + perf mux (jit JTAG)
  // =========================================================================
  assign busy        = mode_simd_eff ? busy_simd : busy_seq;
  assign done        = mode_simd_eff ? done_simd : done_seq;
  assign perf_flops  = mode_simd_eff ? perf_flops_simd : perf_flops_seq;
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
