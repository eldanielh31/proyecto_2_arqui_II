// ============================================================================
// top_dsa_seq.sv — Top con control local (switch) y/o remoto (Virtual JTAG)
// Modo HW (por defecto):   USE_VJTAG=1  -> usa vJTAG + jtag_connect
// Modo SIM/TB (opcional):  USE_VJTAG=0  -> ignora vJTAG y usa parámetros *_INIT
// ============================================================================

`timescale 1ps/1ps
`define MEM_INIT_FILE "img_in_64x64.hex"

module top_dsa_seq
#(
  parameter int AW                = 12,
  // ---- Debounce / reset event ----
  parameter int DEB_W             = 20,      // debounce (~10–20 ms @50MHz)
  parameter int RST_STRETCH_W     = 22,      // pulso visual post-reset
  // ---- Config SIM (solo usadas cuando USE_VJTAG=0) ----
  parameter int unsigned IN_W_INIT      = 16'd64,
  parameter int unsigned IN_H_INIT      = 16'd64,
  parameter int unsigned SCALE_Q88_INIT = 16'd205,
  // ---- Conmutador HW/SIM ----
  parameter bit USE_VJTAG        = 1'b1
)(
  input  logic clk_50,
  input  logic rst_n,          // reset asíncrono activo bajo (de la placa)
  input  logic start_sw,       // switch físico de inicio (nivel)

  output logic led_done,       // latched: ON cuando termina, hasta próximo start/reset
  output logic led_reset_evt,  // ON por ventana corta tras tocar reset
  output logic led_start_on    // refleja el start_sw filtrado/sincronizado
);

  // ---------------- Señales core ----------------
  logic        start_pulse_sw;      // pulso desde switch (debounced)
  logic        start_pulse_jtag;    // pulso desde JTAG (solo si USE_VJTAG=1)
  logic        start_core;          // OR de ambos
  logic        busy, done;

  // Config efectiva hacia el core
  logic [15:0] in_w_cfg, in_h_cfg, scale_q88_cfg;

  logic [15:0] out_w_s, out_h_s;

  logic [AW-1:0] in_raddr;
  logic [7:0]    in_rdata;

  logic [AW-1:0] out_waddr;
  logic [7:0]    out_wdata;
  logic          out_we;

  // =========================================================================
  // 1) Sincronizador + antirrebote simple del switch de start
  // =========================================================================
  logic sw_meta, sw_sync;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      sw_meta <= 1'b0;
      sw_sync <= 1'b0;
    end else begin
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
      if (sw_sync != sw_debounced) begin
        deb_cnt <= '0;
      end else if (deb_cnt != {DEB_W{1'b1}}) begin
        deb_cnt <= deb_cnt + 1'b1;
      end
      if (deb_cnt == {DEB_W{1'b1}}) begin
        sw_debounced <= sw_sync;
      end
      sw_debounced_q <= sw_debounced;
    end
  end

  // Pulso de start por switch
  assign start_pulse_sw = (sw_debounced & ~sw_debounced_q);
  assign led_start_on   = sw_debounced;

  // =========================================================================
  // 2) LED de evento de reset (latido corto al soltar reset)
  // =========================================================================
  logic [RST_STRETCH_W-1:0] rst_cnt;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      rst_cnt <= {RST_STRETCH_W{1'b1}};
    end else begin
      if (rst_cnt != '0) rst_cnt <= rst_cnt - 1'b1;
    end
  end
  assign led_reset_evt = (rst_cnt != '0);

  // =========================================================================
  // 3) Memorias on-chip
  // =========================================================================
  onchip_mem_img #(
    .ADDR_W (AW),
    .INIT_EN(1'b1)           // cargar imagen desde archivo HEX / MIF
  ) mem_in (
    .clk   (clk_50),
    .raddr (in_raddr),
    .rdata (in_rdata),
    .waddr ('0),
    .wdata ('0),
    .we    (1'b0)
  );

  onchip_mem_img #(
    .ADDR_W (AW),
    .INIT_EN(1'b0)
  ) mem_out (
    .clk   (clk_50),
    .raddr ('0),
    .rdata (),
    .waddr (out_waddr),
    .wdata (out_wdata),
    .we    (out_we)
  );

  // =========================================================================
  // 4) Configuración efectiva: vJTAG (HW) o parámetros (SIM)
  // =========================================================================
  generate
    if (USE_VJTAG) begin : G_VJTAG
      // Señales vJTAG con keep/preserve para evitar optimización
      (* keep = 1, preserve = 1 *) wire       vj_tck;
      (* keep = 1, preserve = 1 *) wire       vj_tdi;
      (* keep = 1, preserve = 1 *) wire       vj_tdo;
      (* keep = 1, preserve = 1 *) wire [1:0] vj_ir_in;
      (* keep = 1, preserve = 1 *) wire [1:0] vj_ir_out;
      (* keep = 1, preserve = 1 *) wire       vj_cdr, vj_sdr, vj_udr;
      wire vj_e1dr, vj_pdr, vj_e2dr, vj_cir, vj_uir; // no usados

      // IP vJTAG
      vjtag u_vjtag (
        .tdi                (vj_tdi),
        .tdo                (vj_tdo),
        .ir_in              (vj_ir_in),
        .ir_out             (vj_ir_out),
        .virtual_state_cdr  (vj_cdr),
        .virtual_state_sdr  (vj_sdr),
        .virtual_state_e1dr (vj_e1dr),
        .virtual_state_pdr  (vj_pdr),
        .virtual_state_e2dr (vj_e2dr),
        .virtual_state_udr  (vj_udr),
        .virtual_state_cir  (vj_cir),
        .virtual_state_uir  (vj_uir),
        .tck                (vj_tck)
      );

      // Puente CDC + banco de registros
      wire [15:0] cfg_w, cfg_h, cfg_s;
      jtag_connect #(.DRW(40)) u_jtag_bridge (
        .tck          (vj_tck),
        .tdi          (vj_tdi),
        .tdo          (vj_tdo),
        .ir_in        (vj_ir_in),
        .ir_out       (vj_ir_out),
        .vs_cdr       (vj_cdr),
        .vs_sdr       (vj_sdr),
        .vs_udr       (vj_udr),

        .start_pulse  (start_pulse_jtag),
        .cfg_in_w     (cfg_w),
        .cfg_in_h     (cfg_h),
        .cfg_scale_q88(cfg_s),
        .status_done  (done),

        .clk_sys      (clk_50),
        .rst_sys_n    (rst_n)
      );

      assign in_w_cfg      = cfg_w;
      assign in_h_cfg      = cfg_h;
      assign scale_q88_cfg = cfg_s;

    end else begin : G_SIM
      // En simulación, usar parámetros *_INIT y desactivar start por JTAG
      assign in_w_cfg      = IN_W_INIT[15:0];
      assign in_h_cfg      = IN_H_INIT[15:0];
      assign scale_q88_cfg = SCALE_Q88_INIT[15:0];
      assign start_pulse_jtag = 1'b0;
    end
  endgenerate

  // =========================================================================
  // 5) Núcleo bilineal
  // =========================================================================
  assign start_core = start_pulse_sw | start_pulse_jtag;

  bilinear_seq #(.AW(AW)) core (
    .clk         (clk_50),
    .rst_n       (rst_n),
    .start       (start_core),
    .busy        (busy),
    .done        (done),

    .i_in_w      (in_w_cfg),
    .i_in_h      (in_h_cfg),
    .i_scale_q88 (scale_q88_cfg),

    .o_out_w     (out_w_s),
    .o_out_h     (out_h_s),

    .in_raddr    (in_raddr),
    .in_rdata    (in_rdata),

    .out_waddr   (out_waddr),
    .out_wdata   (out_wdata),
    .out_we      (out_we)
  );

  // =========================================================================
  // 6) LED done latcheado
  // =========================================================================
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      led_done <= 1'b0;
    end else begin
      if (start_core)
        led_done <= 1'b0;
      else if (done)
        led_done <= 1'b1;
    end
  end

endmodule
