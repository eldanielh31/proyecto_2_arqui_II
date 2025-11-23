`timescale 1ps/1ps

// ============================================================================
// tb_bilinear_seq — Testbench para top_dsa_seq
//   - Ejecuta primero el núcleo SECUENCIAL (mode_simd_sw = 0).
//   - Luego hace reset y ejecuta el núcleo SIMD4 (mode_simd_sw = 1).
//   - Genera clock, reset y pulso de start (forzando start_pulse_sw interno).
//   - Carga la imagen de entrada 10x10 en mem_in desde un archivo HEX.
//   - Espera a 'done' y vuelca mem_out a archivos HEX (seq y simd).
//   - Imprime contadores de performance y logs internos de ambos núcleos.
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

  // Switch físico de start (no se usa directamente, se forza start_pulse_sw)
  logic start_sw;

  // Switch físico para modo SIMD por hardware
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

  // Identificador de corrida: 0 = ninguna, 1 = seq, 2 = simd
  int run_id;

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
  // Task: volcar mem_out a archivo HEX
  //   - Usa acceso jerárquico a dut.mem_out.mem[] (onchip_mem_dp).
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

        if (addr < (1 << AW))
          val = dut.mem_out.mem[addr];
        else
          val = 8'h00;

        $fdisplay(fd, "%02h", val);
      end
    end

    $fclose(fd);
    $display("[TB] Dump completado.");
  end
  endtask

  // --------------------------------------------------------------------------
  // Secuencia principal: reset, SEQ, reset, SIMD4
  // --------------------------------------------------------------------------
  initial begin
    integer fd_check;
    integer i;

    rst_n        = 1'b0;
    start_sw     = 1'b0;
    mode_simd_sw = 1'b0;
    run_id       = 0;

    // Reset inicial
    #100_000;              // 100 ns
    rst_n = 1'b1;

    // Esperar a fin de limpieza de mem_out del top
    @(negedge dut.clear_active);
    repeat (5) @(posedge clk_50);

    // Forzar parámetros 10x10, escala 205 Q8.8
    force dut.in_w_cfg      = 16'd10;
    force dut.in_h_cfg      = 16'd10;
    force dut.scale_q88_cfg = 16'd205;
    $display("[TB] Forzando in_w_cfg=10, in_h_cfg=10, scale_q88_cfg=205 (Q8.8)");

    // Verificar y cargar imagen de entrada en mem_in
    fd_check = $fopen(IN_HEX_FILE, "r");
    if (fd_check == 0) begin
      $display("[TB][ERROR] No se pudo abrir '%s' para lectura.", IN_HEX_FILE);
    end else begin
      $fclose(fd_check);
      $display("[TB] Cargando imagen de entrada desde '%s' en mem_in...", IN_HEX_FILE);
      $readmemh(IN_HEX_FILE, dut.mem_in.mem);

      for (i = 0; i < 16; i = i + 1) begin
        $display("[TB] mem_in[%0d] = 0x%02h", i, dut.mem_in.mem[i]);
      end
    end

    // ======================================================
    // CORRIDA 1: Núcleo SECUENCIAL (mode_simd_sw = 0)
    // ======================================================
    run_id       = 1;
    mode_simd_sw = 1'b0;  // 0 = secuencial

    $display("\n[TB] ===== INICIO CORRIDA 1: SECUENCIAL (10x10) ===== t=%0t\n", $time);

    // Generar pulso en start_pulse_sw interno del top
    $display("[TB][SEQ] Forzando start_pulse_sw en t=%0t", $time);
    force dut.start_pulse_sw = 1'b1;
    @(posedge clk_50);
    force dut.start_pulse_sw = 1'b0;
    @(posedge clk_50);
    release dut.start_pulse_sw;

    // Esperar a que el top reporte done (núcleo seleccionado)
    @(posedge dut.done);

    $display("[TB][SEQ] DONE detectado en t=%0t", $time);
    $display("[TB][SEQ] mode_simd_eff = %0b (0=SEQ,1=SIMD4)", dut.mode_simd_eff);

    ow = dut.out_w_s_seq;
    oh = dut.out_h_s_seq;

    $display("[TB][SEQ] out_w = %0d, out_h = %0d", ow, oh);
    $display("[TB][SEQ] perf: flops=%0d mem_rd=%0d mem_wr=%0d",
             dut.perf_flops_seq,
             dut.perf_mem_rd_seq,
             dut.perf_mem_wr_seq);

    dump_mem_out(
      "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_out_seq.hex",
      ow, oh
    );

    // ======================================================
    // Reset entre corridas
    // ======================================================
    $display("\n[TB] ===== RESETEANDO PARA CORRIDA 2: SIMD4 (10x10) ===== t=%0t\n", $time);

    rst_n = 1'b0;
    #100_000;
    rst_n = 1'b1;

    // Esperar limpiador de mem_out
    @(negedge dut.clear_active);
    repeat (5) @(posedge clk_50);

    // Reaplicar parámetros 10x10, 205 Q8.8 (por claridad)
    force dut.in_w_cfg      = 16'd10;
    force dut.in_h_cfg      = 16'd10;
    force dut.scale_q88_cfg = 16'd205;
    $display("[TB] (Corrida 2) Forzando in_w_cfg=10, in_h_cfg=10, scale_q88_cfg=205 (Q8.8)");

    // ======================================================
    // CORRIDA 2: Núcleo SIMD4 (mode_simd_sw = 1)
    // ======================================================
    run_id       = 2;
    mode_simd_sw = 1'b1;  // SIMD4

    $display("\n[TB] ===== INICIO CORRIDA 2: SIMD4 (10x10) ===== t=%0t\n", $time);

    $display("[TB][SIMD4] Forzando start_pulse_sw en t=%0t", $time);
    force dut.start_pulse_sw = 1'b1;
    @(posedge clk_50);
    force dut.start_pulse_sw = 1'b0;
    @(posedge clk_50);
    release dut.start_pulse_sw;

    @(posedge dut.done);

    $display("[TB][SIMD4] DONE detectado en t=%0t", $time);
    $display("[TB][SIMD4] mode_simd_eff = %0b (0=SEQ,1=SIMD4)", dut.mode_simd_eff);

    ow = dut.out_w_s_simd;
    oh = dut.out_h_s_simd;

    $display("[TB][SIMD4] out_w = %0d, out_h = %0d", ow, oh);
    $display("[TB][SIMD4] perf: flops=%0d mem_rd=%0d mem_wr=%0d",
             dut.perf_flops_simd,
             dut.perf_mem_rd_simd,
             dut.perf_mem_wr_simd);

    dump_mem_out(
      "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_out_simd.hex",
      ow, oh
    );

    $display("[TB] Simulación terminada correctamente.");
    $finish;
  end

  // --------------------------------------------------------------------------
  // Watchdog (timeout global sobre las dos corridas)
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
  // Monitor SECUENCIAL: logs solo desde el TB
  // --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    if (!dut.mode_simd_eff && dut.out_we_core) begin
      $display("[SEQ][RUN=%0d][t=%0t] WRITE ox=%0d oy=%0d addr=%0d pix=0x%02h | sx_fix=%0d.%0d sy_fix=%0d.%0d | xi=%0d yi=%0d fx_q=%0d fy_q=%0d I00=%0d I10=%0d I01=%0d I11=%0d",
        run_id,
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
        dut.u_core_seq.fy_q,
        dut.u_core_seq.I00,
        dut.u_core_seq.I10,
        dut.u_core_seq.I01,
        dut.u_core_seq.I11
      );
    end
  end

  // --------------------------------------------------------------------------
  // Monitor SIMD4: logs solo desde el TB
  //   - Se imprime una línea por escritura (1 lane por ciclo).
  //   - Muestra coordenadas de fila, group_ox, lanes y pesos.
  // --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    if (dut.mode_simd_eff && dut.out_we_core) begin
      // Determinar qué lane se está escribiendo según el estado interno
      automatic int lane_sel;
      lane_sel = 0;
      unique case (dut.u_core_simd4.state)
        dut.u_core_simd4.S_WRITE0: lane_sel = 0;
        dut.u_core_simd4.S_WRITE1: lane_sel = 1;
        dut.u_core_simd4.S_WRITE2: lane_sel = 2;
        dut.u_core_simd4.S_WRITE3: lane_sel = 3;
        default:                    lane_sel = -1;
      endcase

      if (lane_sel >= 0 && lane_sel < 4) begin
        $display("[SIMD][RUN=%0d][t=%0t] WRITE lane=%0d oy=%0d group_ox=%0d addr=%0d pix=0x%02h | lane_x={%0d,%0d,%0d,%0d} | xi={%0d,%0d,%0d,%0d} yi_row=%0d | fx_q={%0d,%0d,%0d,%0d} fy_q_row=%0d",
          run_id,
          $time,
          lane_sel,
          dut.u_core_simd4.oy_cur,
          dut.u_core_simd4.group_ox,
          dut.out_waddr_core,
          dut.out_wdata_core,
          dut.u_core_simd4.lane_x[0], dut.u_core_simd4.lane_x[1],
          dut.u_core_simd4.lane_x[2], dut.u_core_simd4.lane_x[3],
          dut.u_core_simd4.xi_base[0], dut.u_core_simd4.xi_base[1],
          dut.u_core_simd4.xi_base[2], dut.u_core_simd4.xi_base[3],
          dut.u_core_simd4.yi_base_row,
          dut.u_core_simd4.fx_q[0], dut.u_core_simd4.fx_q[1],
          dut.u_core_simd4.fx_q[2], dut.u_core_simd4.fx_q[3],
          dut.u_core_simd4.fy_q_row
        );
      end
    end
  end

endmodule
