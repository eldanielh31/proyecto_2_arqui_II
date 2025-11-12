`timescale 1ns/1ps
import fixed_pkg::*;

module top_dsa_seq(
  input  logic clk_50,    // CLOCK_50 (50 MHz)
  input  logic rst_n,     // reset activo alto (invertir afuera si KEY es activo-bajo)
  input  logic start_sw,  // switch opcional para iniciar desde la placa
  output logic led_done   // LED "done"
);
  localparam ADDR_W = 19;

  // Señales núcleo
  logic              busy, done;
  logic [ADDR_W-1:0] in_addr;
  logic [7:0]        in_data;
  logic [ADDR_W-1:0] out_addr;
  logic [7:0]        out_data;
  logic              out_we;

  // ========= Virtual JTAG =========
  // Asegúrese de incluir vjtag.qip en el proyecto (NO incluir vjtag_inst.v).
  wire        v_tck, v_tdi, v_tdo;
  wire [1:0]  v_ir_in, v_ir_out;
  wire        v_cdr, v_sdr, v_udr, v_cir;

  vjtag u_vjtag (
    .tck (v_tck),
    .tdi (v_tdi),
    .tdo (v_tdo),
    .ir_in  (v_ir_in),
    .ir_out (v_ir_out),
    .virtual_state_cdr (v_cdr),
    .virtual_state_sdr (v_sdr),
    .virtual_state_udr (v_udr),
    .virtual_state_cir (v_cir)
    // Si el IP genera más virtual_state_* puede dejarlos sin conectar
  );

  // Puente JTAG ↔ registros del core
  wire        j_start;
  wire [15:0] j_in_w, j_in_h, j_scale;

  jtag_connect u_jc (
    .tck        (v_tck),
    .tdi        (v_tdi),
    .tdo        (v_tdo),
    .ir_in      (v_ir_in),
    .ir_out     (v_ir_out),
    .vs_cdr     (v_cdr),
    .vs_sdr     (v_sdr),
    .vs_udr     (v_udr),

    .start_pulse    (j_start),
    .cfg_in_w       (j_in_w),
    .cfg_in_h       (j_in_h),
    .cfg_scale_q88  (j_scale),
    .status_done    (done),

    .clk_sys    (clk_50),
    .rst_sys_n  (rst_n)
  );

  // ========= Memorias on-chip =========
  onchip_mem_img #(.ADDR_W(ADDR_W)) mem_in (
    .clk   (clk_50),
    .raddr (in_addr),
    .rdata (in_data),
    .waddr ('0),
    .wdata ('0),
    .we    (1'b0)
  );

  onchip_mem_img #(.ADDR_W(ADDR_W)) mem_out (
    .clk   (clk_50),
    .raddr ('0),
    .rdata (),
    .waddr (out_addr),
    .wdata (out_data),
    .we    (out_we)
  );

  // ========= Arranque por JTAG O por switch =========
  logic start_sw_sync;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) start_sw_sync <= 1'b0;
    else        start_sw_sync <= start_sw;
  end
  wire start_core = j_start | start_sw_sync;

  // ========= Núcleo secuencial =========
  bilinear_seq #(.ADDR_W(ADDR_W)) core (
    .clk       (clk_50),
    .rst_n     (rst_n),
    .in_w      (j_in_w),
    .in_h      (j_in_h),
    .scale_q88 (j_scale),
    .start     (start_core),
    .busy      (busy),
    .done      (done),

    .in_addr   (in_addr),
    .in_data   (in_data),
    .out_addr  (out_addr),
    .out_data  (out_data),
    .out_we    (out_we)
  );

  // LED de estado
  assign led_done = done;

  // (Opcional, solo simulación) $readmemh para mem_in:
  // initial $readmemh("img_in_64x64.hex", mem_in.mem);

endmodule
