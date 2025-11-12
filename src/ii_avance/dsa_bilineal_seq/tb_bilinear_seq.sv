// ============================================================================
// tb_bilinear_seq.sv — Testbench (ModelSim/Questa). No sintetizable.
// ============================================================================

`timescale 1ps/1ps

module tb_bilinear_seq;

  localparam AW = 12;
  localparam int DEPTH = (1<<AW);

  logic clk;
  logic rst_n;

  // DUT
  top_dsa_seq #(.AW(AW)) dut (
    .clk_50 (clk),
    .rst_n  (rst_n)
  );

  // Clock 50MHz (periodo 20ns)
  initial begin
    clk = 1'b0;
    forever #10_000 clk = ~clk; // 50 MHz
  end

  // Reset
  initial begin
    rst_n = 1'b0;
    #100_000; // 100 ns
    rst_n = 1'b1;
  end

  // Carga de imagen con validación y fallback
  localparam string IMG_FILE = "C:/Users/danbg/src/proyecto_2_arqui_II/src/ii_avance/dsa_bilineal_seq/img_in_64x64.hex";

  initial begin : load_image
    integer fd_chk;
    integer i;
    integer checksum;

    // Comprobar existencia
    fd_chk = $fopen(IMG_FILE, "r");
    if (fd_chk == 0) begin
      $display("[TB][WARN] No se pudo abrir '%s'. Se cargará un PATRÓN SINTÉTICO.", IMG_FILE);
      for (i = 0; i < DEPTH; i++) begin
        dut.mem_in.mem[i] = ((i % 64) ^ (i / 64)) & 8'hFF;
      end
    end else begin
      $fclose(fd_chk);
      $readmemh(IMG_FILE, dut.mem_in.mem);
      checksum = 0;
      for (i = 0; i < 512; i++) checksum = checksum + dut.mem_in.mem[i];
      if (checksum == 0) begin
        $display("[TB][WARN] '%s' cargó, pero checksum inicial es 0. Se usará patrón sintético.", IMG_FILE);
        for (i = 0; i < DEPTH; i++) begin
          dut.mem_in.mem[i] = ((i % 64) ^ (i / 64)) & 8'hFF;
        end
      end else begin
        $display("[TB] '%s' cargado. Checksum(0..511) = %0d", IMG_FILE, checksum);
      end
    end
  end

  // Monitoreo y volcado de salida (área útil ow*oh)
  initial begin
    $display("[TB] in_w=%0d in_h=%0d scale_q88=%0d",
              dut.core.i_in_w, dut.core.i_in_h, dut.core.i_scale_q88);

    wait(dut.core.done == 1'b1);
    $display("[TB] done=1; out_w=%0d out_h=%0d", dut.core.o_out_w, dut.core.o_out_h);

    dump_mem_to_hex("C:/Users/danbg/src/proyecto_2_arqui_II/src/ii_avance/dsa_bilineal_seq/img_out.hex", dut.core.o_out_w, dut.core.o_out_h);
    $finish;
  end

  task dump_mem_to_hex(string fname, input int ow, input int oh);
    integer fd, x, y, addr;
    begin
      fd = $fopen(fname, "w");
      if (fd == 0) begin
        $display("[TB][ERROR] no se pudo abrir %s", fname);
        disable dump_mem_to_hex;
      end
      for (y = 0; y < oh; y++) begin
        for (x = 0; x < ow; x++) begin
          addr = y * dut.core.o_out_w + x;
          $fdisplay(fd, "%02x", dut.mem_out.mem[addr]);
        end
      end
      $fclose(fd);
      $display("[TB] Archivo %s generado. (pixeles útiles: %0d x %0d)", fname, ow, oh);
    end
  endtask

endmodule
