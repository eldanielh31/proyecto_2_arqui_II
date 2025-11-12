// ============================================================================
// top_dsa_seq.sv
// Top simple: conecta memorias y el núcleo para síntesis/simulación.
// La instancia vJTAG queda comentada para evitar warnings y puertos no usados.
// ============================================================================

`timescale 1ps/1ps

module top_dsa_seq
#(
  parameter AW = 12
)(
  input  logic clk_50,
  input  logic rst_n
);

  localparam IN_W_INIT      = 16'd64;
  localparam IN_H_INIT      = 16'd64;
  localparam SCALE_Q88_INIT = 16'd205; // ~0.80

  // Señales de control
  logic start_pulse;
  logic busy, done;

  // Dimensiones IO
  logic [15:0] in_w, in_h, scale_q88;
  logic [15:0] out_w_s, out_h_s;

  // Memorias on-chip
  logic [AW-1:0] in_raddr;
  logic [7:0]    in_rdata;

  logic [AW-1:0] out_waddr;
  logic [7:0]    out_wdata;
  logic          out_we;

  // Parámetros estáticos (pueden sobreescribirse en TB)
  assign in_w      = IN_W_INIT;
  assign in_h      = IN_H_INIT;
  assign scale_q88 = SCALE_Q88_INIT;

  // Generación de pulso de start
  logic [7:0] start_cnt;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      start_cnt <= 8'd0;
    end else if (start_cnt != 8'd20) begin
      start_cnt <= start_cnt + 8'd1;
    end
  end
  assign start_pulse = (start_cnt == 8'd10);

  // Memoria de entrada (solo lectura)
  onchip_mem_img #(.ADDR_W(AW)) mem_in (
    .clk   (clk_50),
    .raddr (in_raddr),
    .rdata (in_rdata),
    .waddr ('0),
    .wdata ('0),
    .we    (1'b0)
  );

  // Memoria de salida (solo escritura en este top)
  onchip_mem_img #(.ADDR_W(AW)) mem_out (
    .clk   (clk_50),
    .raddr ('0),
    .rdata (),
    .waddr (out_waddr),
    .wdata (out_wdata),
    .we    (out_we)
  );

  // Núcleo bilineal
  bilinear_seq #(.AW(AW)) core (
    .clk         (clk_50),
    .rst_n       (rst_n),
    .start       (start_pulse),
    .busy        (busy),
    .done        (done),
    .i_in_w      (in_w),
    .i_in_h      (in_h),
    .i_scale_q88 (scale_q88),
    .o_out_w     (out_w_s),
    .o_out_h     (out_h_s),
    .in_raddr    (in_raddr),
    .in_rdata    (in_rdata),
    .out_waddr   (out_waddr),
    .out_wdata   (out_wdata),
    .out_we      (out_we)
  );

  // (Opcional) vJTAG — comentado para evitar puertos faltantes en sim/síntesis
//  vjtag u_vjtag (
//    .tck(), .tdi(), .tdo(),
//    .virtual_state_cdr(), .virtual_state_sdr(),
//    .virtual_state_e1dr(), .virtual_state_pdr(),
//    .virtual_state_e2dr(), .virtual_state_udr(),
//    .virtual_state_cir(),  .virtual_state_uir(),
//    .ir_in(), .ir_out(),
//    .jtag_state_tlr(), .jtag_state_rti(),
//    .jtag_state_sdrs(), .jtag_state_cdr(),
//    .jtag_state_sdr(), .jtag_state_e1dr(),
//    .jtag_state_pdr(), .jtag_state_e2dr(),
//    .jtag_state_udr(), .jtag_state_sirs(),
//    .jtag_state_cir(), .jtag_state_sir(),
//    .jtag_state_e1ir(), .jtag_state_pir(),
//    .jtag_state_e2ir(), .tms()
//  );

endmodule
