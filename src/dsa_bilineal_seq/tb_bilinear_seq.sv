`timescale 1ps/1ps

// ============================================================================
// tb_bilinear_seq — Testbench para top_dsa_seq
//   - Ejecuta el núcleo en modo secuencial (mode_simd_sw = 0).
//   - Genera clock, reset y pulso de start (forzando start_pulse_sw interno).
//   - Carga la imagen de entrada en mem_in desde un archivo HEX.
//   - Espera a 'done' y vuelca mem_out a un archivo HEX.
//   - Imprime contadores de performance y parte del datapath interno.
// ============================================================================

module tb_bilinear_seq;

  // --------------------------------------------------------------------------
  // Parámetros
  // --------------------------------------------------------------------------
  localparam int    AW          = 12;          // Debe coincidir con top_dsa_seq
  localparam time   T_CLK_PS    = 20_000;      // 20 ns -> 50 MHz
  localparam string IN_HEX_FILE = "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_in_10x10.hex";

  // --------------------------------------------------------------------------
  // Señales hacia el DUT
  // --------------------------------------------------------------------------
  logic clk_50;
  logic rst_n;

  // Puerto de start del top (switch físico).
  // En este TB se mantiene en 0; se usará start_pulse_sw interno vía force.
  logic start_sw;

  // Modo SIMD por switch físico: 0 = secuencial, 1 = SIMD4
  logic mode_simd_sw;

  logic led_done;
  logic led_reset_evt;
  logic led_start_on;
  logic led_simd_mode;

  // --------------------------------------------------------------------------
  // Variables para watchdog y dimensiones de salida
  // --------------------------------------------------------------------------
  longint unsigned cycles;
  longint unsigned MAX_CYCLES;

  int ow;
  int oh;

  // --------------------------------------------------------------------------
  // DUT
  // --------------------------------------------------------------------------
  top_dsa_seq #(
    .AW            (AW),
    .DEB_W         (20),
    .RST_STRETCH_W (22)
  ) dut (
    .clk_50       (clk_50),
    .rst_n        (rst_n),

    .start_sw     (start_sw),
    .mode_simd_sw (mode_simd_sw),

    .led_done     (led_done),
    .led_reset_evt(led_reset_evt),
    .led_start_on (led_start_on),
    .led_simd_mode(led_simd_mode)
  );

  // --------------------------------------------------------------------------
  // Clock 50 MHz
  // --------------------------------------------------------------------------
  initial begin
    clk_50 = 1'b0;
    forever #(T_CLK_PS/2) clk_50 = ~clk_50;
  end

  // --------------------------------------------------------------------------
  // Reset + configuración inicial
  // --------------------------------------------------------------------------
  initial begin
    rst_n        = 1'b0;
    start_sw     = 1'b0;   // NO se usa para generar start en este TB
    mode_simd_sw = 1'b0;   // 0 = núcleo secuencial

    #100_000;              // 100 ns
    rst_n = 1'b1;
  end

  // --------------------------------------------------------------------------
  // Forzado de dimensiones de entrada (caso imagen 10x10)
  //   - Se fuerzan directamente los puertos del núcleo secuencial.
  // --------------------------------------------------------------------------
  initial begin
    @(posedge rst_n);

    // Forzar directamente los puertos del núcleo secuencial
    // (ajusta i_in_w / i_in_h si tu bilinear_seq usa otros nombres).
    force dut.u_core_seq.i_in_w = 10;
    force dut.u_core_seq.i_in_h = 10;

    $display("[TB] Forzando i_in_w=i_in_h=10 en u_core_seq en t=%0t", $time);
  end

  // --------------------------------------------------------------------------
  // Secuencia: esperar reset + clear, cargar imagen en mem_in y lanzar start
  // --------------------------------------------------------------------------
  initial begin
    integer fd_check;
    integer i;

    @(posedge rst_n);

    // Esperar explícitamente a que termine la limpieza de mem_out
    wait (dut.clear_active == 1'b0);
    // Unos ciclos extra por seguridad
    repeat (5) @(posedge clk_50);

    // Verificar que el archivo existe / se puede abrir
    fd_check = $fopen(IN_HEX_FILE, "r");
    if (fd_check == 0) begin
      $display("[TB][ERROR] No se pudo abrir '%s' para lectura.", IN_HEX_FILE);
    end
    else begin
      $fclose(fd_check);
      $display("[TB] Cargando imagen de entrada desde '%s' en mem_in...", IN_HEX_FILE);

      // Carga de la memoria de entrada
      $readmemh(IN_HEX_FILE, dut.mem_in.mem);

      // Opcional: mostrar algunos valores para verificar
      for (i = 0; i < 16; i = i + 1) begin
        $display("[TB] mem_in[%0d] = 0x%02h", i, dut.mem_in.mem[i]);
      end
    end

    // Generar un pulso de un ciclo en start_pulse_sw dentro del top
    $display("[TB] Forzando start_pulse_sw interno en t=%0t (clear_active=0)", $time);
    force dut.start_pulse_sw = 1'b1;
    @(posedge clk_50);
    force dut.start_pulse_sw = 1'b0;
    @(posedge clk_50);
    release dut.start_pulse_sw;
  end

  // --------------------------------------------------------------------------
  // Watchdog (timeout)
  // --------------------------------------------------------------------------
  initial begin
    cycles     = 0;
    MAX_CYCLES = 10_000_000;  // 10M ciclos de 20 ns ~ 0.2 s de sim
    forever begin
      @(posedge clk_50);
      cycles++;
      if (cycles == 1_000_000)
        $display("[TB][INFO] 1M ciclos...");
      if (cycles > MAX_CYCLES) begin
        $fatal(1, "[TB][TIMEOUT] No se recibió 'done' tras %0d ciclos.", cycles);
      end
    end
  end

  // --------------------------------------------------------------------------
  // Task: volcar mem_out a archivo HEX
  //   - Usa acceso jerárquico a dut.mem_out.mem[] (onchip_mem_dp).
  //   - Interpreta los píxeles en orden raster row-major (y * ow + x).
  // --------------------------------------------------------------------------
  task automatic dump_mem_out(
    input string fname,
    input int    ow_local,
    input int    oh_local
  );
    integer fd;
    int x, y;
    int addr;
    reg [7:0] val;
  begin
    fd = $fopen(fname, "w");
    if (fd == 0) begin
      $display("[TB][ERROR] No se pudo abrir '%s' para escritura.", fname);
      disable dump_mem_out;
    end

    $display("[TB] Volcando mem_out (%0dx%0d) a '%s'", ow_local, oh_local, fname);

    for (y = 0; y < oh_local; y = y + 1) begin
      for (x = 0; x < ow_local; x = x + 1) begin
        addr = y * ow_local + x;

        // Acceso directo al array interno de onchip_mem_dp:
        if (addr < (1 << AW))
          val = dut.mem_out.mem[addr];
        else
          val = 8'h00;

        // Un byte por línea en HEX (formato típico $readmemh)
        $fdisplay(fd, "%02h", val);
      end
    end

    $fclose(fd);
    $display("[TB] Dump completado.");
  end
  endtask

  // --------------------------------------------------------------------------
  // Monitor opcional de escrituras del núcleo secuencial
  // --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    // Núcleo secuencial activo
    if (!dut.mode_simd_eff && dut.out_we_core) begin
      $display("[SEQ][t=%0t] WRITE ox=%0d oy=%0d addr=%0d pix=0x%02h | sx_fix=%0d.%0d sy_fix=%0d.%0d | xi=%0d yi=%0d fx_q=%0d fy_q=%0d",
        $time,
        dut.u_core_seq.ox_cur,
        dut.u_core_seq.oy_cur,
        dut.out_waddr_core,
        dut.out_wdata_core,
        dut.u_core_seq.sx_fix[23:8], dut.u_core_seq.sx_fix[7:0],
        dut.u_core_seq.sy_fix[23:8], dut.u_core_seq.sy_fix[7:0],
        dut.u_core_seq.xi_base,
        dut.u_core_seq.yi_base,
        dut.u_core_seq.fx_q,
        dut.u_core_seq.fy_q
      );
    end
  end

  // --------------------------------------------------------------------------
  // Finalización al detectar 'done'
  // --------------------------------------------------------------------------
  initial begin
    @(posedge rst_n);
    wait (dut.done == 1'b1);

    $display("[TB] DONE detectado en t=%0t", $time);
    $display("[TB] mode_simd_eff = %0b (0=SEQ, 1=SIMD4)", dut.mode_simd_eff);

    // Elegir dimensiones según modo activo
    if (!dut.mode_simd_eff) begin
      ow = dut.out_w_s_seq;
      oh = dut.out_h_s_seq;
    end else begin
      ow = dut.out_w_s_simd;
      oh = dut.out_h_s_simd;
    end

    $display("[TB] out_w = %0d, out_h = %0d", ow, oh);
    $display("[TB] perf: flops=%0d mem_rd=%0d mem_wr=%0d",
             dut.perf_flops,
             dut.perf_mem_rd,
             dut.perf_mem_wr);

    dump_mem_out(
      "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_out_tb.hex",
      ow,
      oh
    );

    $display("[TB] Simulación terminada.");
    $finish;
  end

endmodule
