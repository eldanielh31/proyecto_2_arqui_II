// ============================================================================
// tb_bilinear_seq.sv — Testbench (ModelSim/Questa). No sintetizable.
// Define MEM_INIT_FILE en el toplevel o aquí para sim.
// Opcional: define TB_GOLDEN_FILE para comparar contra golden HEX.
// ============================================================================

`timescale 1ps/1ps

`define MEM_INIT_FILE "C:/Users/danbg/src/proyecto_2_arqui_II/src/ii_avance/dsa_bilineal_seq/img_in_64x64.hex"
//`define TB_GOLDEN_FILE "C:/ruta/a/img_out_golden.hex"

module tb_bilinear_seq;

  localparam int AW    = 19;
  localparam int DEPTH = (1<<AW);

  // Clocks & resets
  logic clk;
  logic rst_n;

  // Señales del top
  logic start_sw;
  logic led_done, led_reset_evt, led_start_on;

  // DUT
  top_dsa_seq #(
    .AW(AW),
    .DEB_W(2),
    .RST_STRETCH_W(4)
  ) dut (
    .clk_50       (clk),
    .rst_n        (rst_n),
    .start_sw     (start_sw),
    .led_done     (led_done),
    .led_reset_evt(led_reset_evt),
    .led_start_on (led_start_on)
  );

  // Clock 50MHz (20ns)
  initial begin
    clk = 1'b0;
    forever #10_000 clk = ~clk;
  end

  // Reset asincrónico activo bajo
  initial begin
    rst_n   = 1'b0;
    start_sw = 1'b0;
    #100_000;           // 100 ns
    rst_n   = 1'b1;
  end

  // Estímulo de start con debounce:
  initial begin : drive_start
    @(posedge rst_n);
    repeat (10) @(posedge clk);
    start_sw = 1'b1; repeat (16) @(posedge clk); start_sw = 1'b0;
  end

  // Log básico
  initial begin
    $display("[TB] Arrancando. AW=%0d", AW);
  end

  // Timeout defensivo
  initial begin : watchdog
    longint unsigned cycles = 0;
    longint unsigned MAX_CYCLES = 10_000_000;
    forever begin
      @(posedge clk);
      cycles++;
      if (cycles == 1_000_000)  $display("[TB][INFO] 1M ciclos...");
      if (cycles > MAX_CYCLES) begin
        $fatal(1, "[TB][TIMEOUT] No se recibió 'done' tras %0d ciclos.", cycles);
      end
    end
  end

  // Finalización cuando el core termine; volcar salida a HEX y comparar opcional
  initial begin
    wait (dut.core.done == 1'b1);
    $display("[TB] done=1; out_w=%0d out_h=%0d", dut.core.o_out_w, dut.core.o_out_h);
    dump_mem_out("C:/Users/danbg/src/proyecto_2_arqui_II/src/ii_avance/dsa_bilineal_seq/img_out.hex",
                 dut.core.o_out_w, dut.core.o_out_h);
`ifdef TB_GOLDEN_FILE
    compare_hex("C:/Users/danbg/src/proyecto_2_arqui_II/src/ii_avance/dsa_bilineal_seq/img_out.hex",
                `TB_GOLDEN_FILE);
`endif
    $finish;
  end

  task dump_mem_out(string fname, input int ow, input int oh);
    integer fd, x, y, addr;
    begin
      fd = $fopen(fname, "w");
      if (fd == 0) begin
        $display("[TB][ERROR] no se pudo abrir %s", fname);
        disable dump_mem_out;
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

`ifdef TB_GOLDEN_FILE
  task compare_hex(string outf, string goldf);
    integer f1, f2, ln, r1, r2;
    reg [31:0] b1, b2;
    begin
      f1 = $fopen(outf, "r"); f2 = $fopen(goldf, "r");
      if (f1==0 || f2==0) begin
        $display("[TB][WARN] No se pudo abrir alguno de los archivos para comparar.");
        if (f1) $fclose(f1); if (f2) $fclose(f2);
        disable compare_hex;
      end
      ln = 0;
      while (1) begin
        r1 = $fscanf(f1, "%h\n", b1);
        r2 = $fscanf(f2, "%h\n", b2);
        if (r1<=0 || r2<=0) break;
        ln++;
        if (b1[7:0] !== b2[7:0]) begin
          $fatal(1, "[TB][FAIL] Mismatch en línea %0d: got=%02h exp=%02h", ln, b1[7:0], b2[7:0]);
        end
      end
      $fclose(f1); $fclose(f2);
      $display("[TB][PASS] Coincidencia bit a bit contra golden.");
    end
  endtask
`endif

endmodule
