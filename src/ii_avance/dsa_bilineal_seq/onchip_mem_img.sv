// onchip_mem_img.sv — RAM 8-bit. En síntesis usa altsyncram (DUAL_PORT);
// en simulación usa un modelo simple con array SV.

`timescale 1ns/1ps

module onchip_mem_img #(
  parameter int ADDR_W = 12
)(
  input  logic              clk,
  // read port
  input  logic [ADDR_W-1:0] raddr,
  output logic [7:0]        rdata,
  // write port
  input  logic [ADDR_W-1:0] waddr,
  input  logic [7:0]        wdata,
  input  logic              we
);

  localparam int DEPTH = 1 << ADDR_W;

`ifdef SYNTHESIS
  // ============================================================
  // SÍNTESIS (Quartus): instanciación explícita de altsyncram
  // ============================================================
  // Notas:
  // - DUAL_PORT: puerto A para lectura, puerto B para escritura.
  // - clock0/clock1 atados al mismo clk.
  // - Datos desregistrados (UNREGISTERED) como el modelo simple.
  // - read_during_write_mode_mixed_ports("DONT_CARE") para evitar checks.
  //
  // Requiere librería 'altera_mf' que Quartus incluye por defecto.
  // ============================================================

  wire [7:0] qa_unused;
  wire [7:0] qb_unused;

  altsyncram #(
    .operation_mode                        ("DUAL_PORT"),
    .width_a                               (8),
    .widthad_a                             (ADDR_W),
    .numwords_a                            (DEPTH),
    .outdata_reg_a                         ("UNREGISTERED"),
    .address_aclr_a                        ("NONE"),
    .outdata_aclr_a                        ("NONE"),
    .indata_aclr_a                         ("NONE"),
    .wrcontrol_aclr_a                      ("NONE"),
    .read_during_write_mode_port_a         ("DONT_CARE"),

    .width_b                               (8),
    .widthad_b                             (ADDR_W),
    .numwords_b                            (DEPTH),
    .outdata_reg_b                         ("UNREGISTERED"),
    .address_aclr_b                        ("NONE"),
    .outdata_aclr_b                        ("NONE"),
    .indata_aclr_b                         ("NONE"),
    .wrcontrol_aclr_b                      ("NONE"),
    .read_during_write_mode_port_b         ("DONT_CARE"),

    .byte_size                             (8),
    .read_during_write_mode_mixed_ports    ("DONT_CARE"),
    .power_up_uninitialized                ("FALSE"),
    .lpm_type                              ("altsyncram")
  ) u_ram (
    // reloj
    .clock0     (clk),
    .clock1     (clk),

    // ----- Puerto A: LECTURA -----
    .address_a  (raddr),
    .q_a        (rdata),
    .data_a     (8'h00),
    .wren_a     (1'b0),
    .rden_a     (1'b1),
    .byteena_a  (1'b1),

    // ----- Puerto B: ESCRITURA -----
    .address_b  (waddr),
    .q_b        (qb_unused),
    .data_b     (wdata),
    .wren_b     (we),
    .rden_b     (1'b1),
    .byteena_b  (1'b1),

    // no usados
    .aclr0      (1'b0),
    .aclr1      (1'b0),
    .addressstall_a (1'b0),
    .addressstall_b (1'b0),

    // puertos no declarados por este modo
    .clocken0   (1'b1),
    .clocken1   (1'b1),
    .clocken2   (1'b1),
    .clocken3   (1'b1)
  );

`else
  // ============================================================
  // SIMULACIÓN (ModelSim): modelo simple con array SV
  // ============================================================
  // Permite $readmemh desde el testbench o top.
  // ============================================================

  // Acepta atributos para mantener paridad con síntesis (no afectan sim).
  (* ramstyle = "M10K, no_rw_check" *) logic [7:0] mem [0:DEPTH-1];

  integer k;
  initial begin
    for (k = 0; k < DEPTH; k++) mem[k] = 8'h00;
  end

  always @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    rdata <= mem[raddr];
  end

  // Exponer 'mem' para $readmemh en TB:
  // $readmemh("img_in_64x64.hex", <instancia>.mem);

`endif

endmodule
