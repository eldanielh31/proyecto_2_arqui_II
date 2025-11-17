// onchip_mem_img.sv — RAM 8-bit con init desde HEX/MIF (asume MEM_INIT_FILE definido)
`timescale 1ns/1ps

module onchip_mem_img #(
  parameter int ADDR_W  = 12,
  parameter bit INIT_EN = 1'b0        // 1: cargar archivo de init
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
  // SÍNTESIS (Quartus): altsyncram dual-port con init_file.
  // Se asume que MEM_INIT_FILE SIEMPRE está definido (literal string).
  // ============================================================
  wire [7:0] qb_unused;

  generate
    if (INIT_EN) begin : G_RAM_INIT
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
        .lpm_type                              ("altsyncram"),
        .init_file                             (`MEM_INIT_FILE)
      ) u_ram (
        .clock0     (clk),
        .clock1     (clk),

        .address_a  (raddr),
        .q_a        (rdata),
        .data_a     (8'h00),
        .wren_a     (1'b0),
        .rden_a     (1'b1),
        .byteena_a  (1'b1),

        .address_b  (waddr),
        .q_b        (qb_unused),
        .data_b     (wdata),
        .wren_b     (we),
        .rden_b     (1'b1),
        .byteena_b  (1'b1),

        .aclr0      (1'b0),
        .aclr1      (1'b0),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0),

        .clocken0   (1'b1),
        .clocken1   (1'b1),
        .clocken2   (1'b1),
        .clocken3   (1'b1)
      );
    end else begin : G_RAM_NOINIT
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
        .lpm_type                              ("altsyncram"),
        .init_file                             ("")
      ) u_ram (
        .clock0     (clk),
        .clock1     (clk),
        .address_a  (raddr), .q_a (rdata), .data_a (8'h00), .wren_a (1'b0), .rden_a (1'b1), .byteena_a (1'b1),
        .address_b  (waddr), .q_b (qb_unused), .data_b (wdata), .wren_b (we), .rden_b (1'b1), .byteena_b (1'b1),
        .aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0),
        .clocken0(1'b1), .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1)
      );
    end
  endgenerate

`else
  // ============================================================
  // SIMULACIÓN (ModelSim): array SV + $readmemh del MEM_INIT_FILE
  // ============================================================
  (* ramstyle = "M10K, no_rw_check" *) logic [7:0] mem [0:DEPTH-1];

  initial begin
    $display("onchip_mem_img: $readmemh from %s", `MEM_INIT_FILE);
    $readmemh(`MEM_INIT_FILE, mem);
  end

  always @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    rdata <= mem[raddr];
  end
`endif

endmodule
