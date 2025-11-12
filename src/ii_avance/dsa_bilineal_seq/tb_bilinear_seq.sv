// ============================================================================
// tb_bilinear_seq.sv — Testbench (ModelSim/Questa). No sintetizable.
// ============================================================================

`timescale 1ps/1ps

// Haz que el modelo de RAM también haga $readmemh en simulación:
`define MEM_INIT_FILE "C:/Users/danbg/src/proyecto_2_arqui_II/src/ii_avance/dsa_bilineal_seq/img_in_64x64.hex"

module tb_bilinear_seq;

  localparam int AW    = 12;
  localparam int DEPTH = (1<<AW);

  // Clocks & resets
  logic clk;
  logic rst_n;

  // Señales del top
  logic start_sw;
  logic led_done, led_reset_evt, led_start_on;

  // DUT: usa overrides pequeños para sim rápida
  top_dsa_seq #(
    .AW(AW),
    .DEB_W(2),           // debounce corto en sim
    .RST_STRETCH_W(4),   // pulso de reset corto en sim
    .IN_W_INIT(16'd64),
    .IN_H_INIT(16'd64),
    .SCALE_Q88_INIT(16'd205)
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

  // Carga de imagen en la RAM de entrada del DUT (vía array de sim)
  initial begin : load_image
    integer fd_chk, i, checksum;
    string  IMG_FILE = `MEM_INIT_FILE;

    // Espera a que exista la instancia mem_in/mem[] (post-elaboración)
    #1;

`ifndef SYNTHESIS
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
      for (i = 0; i < 512; i++) checksum += dut.mem_in.mem[i];
      $display("[TB] '%s' cargado. Checksum(0..511) = %0d", IMG_FILE, checksum);
    end
`endif
  end

  // Estímulo de start con debounce:
  // Mantener el switch en '1' el tiempo suficiente para pasar por
  // 2 FFs de sincronización + conteo de debounce (DEB_W=2 => muy corto).
  initial begin : drive_start
    // Espera a salir de reset y a que el pulso visual de reset pase
    @(posedge rst_n);
    repeat (10) @(posedge clk);

    // Sube el switch; mantenerlo estable algunos ciclos
    start_sw = 1'b1;
    repeat (16) @(posedge clk);

    // Opcional: bajar el switch (no afecta porque el pulso ya se generó)
    start_sw = 1'b0;
  end

  // Log básico
  initial begin
    $display("[TB] in_w=%0d in_h=%0d scale_q88=%0d",
              dut.core.i_in_w, dut.core.i_in_h, dut.core.i_scale_q88);
  end

  // Timeout defensivo para evitar bloqueos silenciosos
  initial begin : watchdog
    longint unsigned cycles = 0;
    longint unsigned MAX_CYCLES = 5_000_000; // ~100 ms simulado a 50 MHz
    forever begin
      @(posedge clk);
      cycles++;
      if (cycles == 1_000)      $display("[TB][INFO] 1k ciclos...");
      if (cycles == 100_000)    $display("[TB][INFO] 100k ciclos...");
      if (cycles == 1_000_000)  $display("[TB][INFO] 1M ciclos...");
      if (cycles > MAX_CYCLES) begin
        $fatal(1, "[TB][TIMEOUT] No se recibió 'done' tras %0d ciclos. Revisar start/estado.", cycles);
      end
    end
  end

  // Finalización cuando el core termine; volcar salida útil a HEX
  initial begin
    wait (dut.core.done == 1'b1);
    $display("[TB] done=1; out_w=%0d out_h=%0d", dut.core.o_out_w, dut.core.o_out_h);
    dump_mem_to_hex("C:/Users/danbg/src/proyecto_2_arqui_II/src/ii_avance/dsa_bilineal_seq/img_out.hex",
                    dut.core.o_out_w, dut.core.o_out_h);
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
