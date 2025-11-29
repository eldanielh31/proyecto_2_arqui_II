
`timescale 1ps/1ps

// ============================================================================
// top_dsa_wide.sv
//   - Top module with wide memory architecture (versión sin JTAG)
//   - Memoria de entrada ancha (32 bits, 4 píxeles/word) con handshake
//   - Sequential y SIMD4 comparten la misma memoria de entrada
// ============================================================================

`define MEM_INIT_FILE "C:/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_16x16.hex"

module top_dsa_seq #(

  // Tamaño de imagen parametrizable
  parameter int IMG_W = 16,
  parameter int IMG_H = 16,

  // AW ahora se interpreta como ancho de dirección en PALABRAS (32 bits)
  parameter int AW = 18,
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
  // Configuración fija (sin JTAG)
  // =========================================================================
  logic [15:0] in_w;
  logic [15:0] in_h;
  logic [15:0] scale_q88;

  // Escala 1.0 en Q8.8 (0x0100); ajustar si se necesita otro factor
  localparam logic [15:0] CFG_IN_W      = IMG_W;
  localparam logic [15:0] CFG_IN_H      = IMG_H;
  localparam logic [15:0] CFG_SCALE_Q88 = 16'd205;

  assign in_w      = CFG_IN_W;
  assign in_h      = CFG_IN_H;
  assign scale_q88 = CFG_SCALE_Q88;

  // Modo SIMD solo desde switch
  logic mode_simd_eff;
  assign mode_simd_eff = mode_simd_sw;
  assign led_simd_mode = mode_simd_eff;

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

  wire start_pulse_sw = (sw_debounced & ~sw_debounced_q);
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
  // Señales de control global start/busy/done
  // =========================================================================
  logic start_raw;
  logic start_any;
  logic busy_seq,  done_seq;
  logic busy_simd, done_simd;
  logic busy,      done;

  // En HW: start desde botón (debounce)
  assign start_raw = (SIMULATION) ? start_sw : start_pulse_sw;
  assign start_any = start_raw; // sin clear de memoria de salida en esta versión

  assign busy = mode_simd_eff ? busy_simd : busy_seq;
  assign done = mode_simd_eff ? done_simd : done_seq;

  // =========================================================================
  // Memoria de entrada: wide_onchip_mem (handshake)
//   - 32 bits = 4 píxeles por palabra
//   - Sólo lectura en este top (req_we = 0)
//   - Compartida entre secuencial y SIMD, pero nunca activos al mismo tiempo
  // =========================================================================
  // Señales hacia la RAM
  logic              mem_in_req_valid;
  logic              mem_in_req_ready;
  logic [AW-1:0]     mem_in_req_addr;
  logic              mem_in_resp_valid;
  logic [31:0]       mem_in_resp_rdata;

  wide_onchip_mem #(
    .ADDR_W (AW),
    .INIT_EN(1'b1)
  ) mem_in (
    .clk        (clk_50),
    .rst_n      (rst_n),
    .req_valid  (mem_in_req_valid),
    .req_ready  (mem_in_req_ready),
    .req_we     (1'b0),            // sólo lectura
    .req_addr   (mem_in_req_addr),
    .req_wdata  (32'h0000_0000),
    .resp_valid (mem_in_resp_valid),
    .resp_rdata (mem_in_resp_rdata)
  );

  // =========================================================================
  // Interfaces de memoria hacia los cores
  // =========================================================================
  // Secuencial
  logic              in_req_valid_seq;
  logic              in_req_ready_seq;
  logic [AW-1:0]     in_req_addr_seq;
  logic              in_resp_valid_seq;
  logic [31:0]       in_resp_rdata_seq;

  // SIMD4
  logic              in_req_valid_simd;
  logic              in_req_ready_simd;
  logic [AW-1:0]     in_req_addr_simd;
  logic              in_resp_valid_simd;
  logic [31:0]       in_resp_rdata_simd;

  // Mux de memoria según modo (sólo un core activo)
  assign mem_in_req_valid = mode_simd_eff ? in_req_valid_simd : in_req_valid_seq;
  assign mem_in_req_addr  = mode_simd_eff ? in_req_addr_simd  : in_req_addr_seq;

  always_comb begin
    if (mode_simd_eff) begin
      // SIMD conectado a la RAM
      in_req_ready_simd   = mem_in_req_ready;
      in_resp_valid_simd  = mem_in_resp_valid;
      in_resp_rdata_simd  = mem_in_resp_rdata;
      // Secuencial desconectado
      in_req_ready_seq    = 1'b0;
      in_resp_valid_seq   = 1'b0;
      in_resp_rdata_seq   = 32'h0000_0000;
    end else begin
      // Secuencial conectado a la RAM
      in_req_ready_seq    = mem_in_req_ready;
      in_resp_valid_seq   = mem_in_resp_valid;
      in_resp_rdata_seq   = mem_in_resp_rdata;
      // SIMD desconectado
      in_req_ready_simd   = 1'b0;
      in_resp_valid_simd  = 1'b0;
      in_resp_rdata_simd  = 32'h0000_0000;
    end
  end

  // =========================================================================
  // Instancias de cores
  // =========================================================================
  // Señales de escritura de salida (no conectadas a RAM en esta versión)
  logic [AW-1:0] out_waddr_seq;
  logic  [7:0]   out_wdata_seq;
  logic          out_we_seq;

  logic [AW-1:0] out_waddr_simd0, out_waddr_simd1, out_waddr_simd2, out_waddr_simd3;
  logic  [7:0]   out_wdata_simd0, out_wdata_simd1, out_wdata_simd2, out_wdata_simd3;
  logic          out_we_simd0,    out_we_simd1,    out_we_simd2,    out_we_simd3;

  // Start gateados por modo
  logic start_seq, start_simd;
  assign start_seq  = start_any & ~mode_simd_eff;
  assign start_simd = start_any &  mode_simd_eff;

  // Secuencial
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

    // Interfaz de memoria de entrada (handshake, 1 puerto)
    .in_req_valid (in_req_valid_seq),
    .in_req_ready (in_req_ready_seq),
    .in_req_addr  (in_req_addr_seq),
    .in_resp_valid(in_resp_valid_seq),
    .in_resp_rdata(in_resp_rdata_seq),

    // Escritura de salida (no conectada a RAM en este top)
    .out_waddr    (out_waddr_seq),
    .out_wdata    (out_wdata_seq),
    .out_we       (out_we_seq),

    .o_flop_count (),
    .o_mem_rd_count(),
    .o_mem_wr_count()
  );

  // SIMD4
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

    // Interfaz de memoria de entrada (handshake, 1 puerto)
    .in_req_valid (in_req_valid_simd),
    .in_req_ready (in_req_ready_simd),
    .in_req_addr  (in_req_addr_simd),
    .in_resp_valid(in_resp_valid_simd),
    .in_resp_rdata(in_resp_rdata_simd),

    // Escritura de salida (no conectada a RAM en este top)
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

    .o_flop_count (),
    .o_mem_rd_count(),
    .o_mem_wr_count()
  );

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
