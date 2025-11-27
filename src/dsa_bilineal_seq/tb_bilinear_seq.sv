`timescale 1ps/1ps

// ============================================================================
// tb_bilinear_seq — Testbench para top_dsa_seq
//   - Ejecuta primero el núcleo SECUENCIAL (mode_simd_sw = 0).
//   - Luego hace reset y ejecuta el núcleo SIMD4 (mode_simd_sw = 1).
//   - Genera clock, reset y pulso de start (forzando start_pulse_sw interno).
//   - Carga la imagen de entrada IN_W_CFG x IN_H_CFG en mem_in desde un archivo HEX.
//   - Copia esa misma imagen a mem_in1, mem_in2, mem_in3 (bancos SIMD).
//   - Espera a 'done' y vuelca mem_out a archivos HEX (seq y simd).
//   - Compara mem_out de SEQ vs mem_out de SIMD4 (golden vs DUT SIMD).
//   - Monitores de debug para SEQ y SIMD, limitados a las primeras direcciones.
// ============================================================================

module tb_bilinear_seq;

  // --------------------------------------------------------------------------
  // Parámetros
  // --------------------------------------------------------------------------
  localparam int    AW             = 12;          // Debe coincidir con top_dsa_seq
  localparam int    MEM_DEPTH      = (1 << AW);
  localparam time   T_CLK_PS       = 20_000;      // 20 ns -> 50 MHz

  // Configuración de imagen / escala
  localparam int    IN_W_CFG       = 16;          // ancho de la imagen de entrada
  localparam int    IN_H_CFG       = 16;          // alto de la imagen de entrada
  localparam int    SCALE_Q88_CFG  = 16'd205;     // factor Q8.8 (~0.8); ajustar si se desea

  localparam string IN_HEX_FILE    = "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_16x16.hex";

  // Flags de debug
  localparam bit DBG_SEQ_MONITOR      = 1'b1;   // Monitores detallados del núcleo secuencial
  localparam bit DBG_SIMD_SUMMARY     = 1'b1;   // Resumen por WRITE en lanes SIMD
  localparam bit DBG_SIMD_DETAILED    = 1'b1;   // MEMCHK/ADDRCHK detallado por lane SIMD
  localparam int DBG_SIMD_MAX_MISM    = 50;     // Máx. mismatches a mostrar en comparación final

  // --------------------------------------------------------------------------
  // Señales hacia el DUT
  // --------------------------------------------------------------------------
  logic clk_50;
  logic rst_n;

  logic start_sw;
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
  // Buffer golden para salida SECUENCIAL
  // --------------------------------------------------------------------------
  logic [7:0] golden_seq [0:MEM_DEPTH-1];

  // --------------------------------------------------------------------------
  // Variables auxiliares para monitores (declaradas a nivel de módulo
  // para evitar errores de sintaxis en ModelSim 10.5).
  // --------------------------------------------------------------------------
  // SECUENCIAL
  integer seq_addr00, seq_addr10, seq_addr01, seq_addr11;
  integer seq_exp_addr;
  integer seq_mem00, seq_mem10, seq_mem01, seq_mem11;

  // SIMD Lane 0
  integer l0_addr00, l0_addr10, l0_addr01, l0_addr11;
  integer l0_exp_addr;
  integer l0_mem00, l0_mem10, l0_mem01, l0_mem11;
  integer l0_valid_lane;

  // SIMD Lane 1
  integer l1_addr00, l1_addr10, l1_addr01, l1_addr11;
  integer l1_exp_addr;
  integer l1_mem00, l1_mem10, l1_mem01, l1_mem11;
  integer l1_valid_lane;

  // SIMD Lane 2
  integer l2_addr00, l2_addr10, l2_addr01, l2_addr11;
  integer l2_exp_addr;
  integer l2_mem00, l2_mem10, l2_mem01, l2_mem11;
  integer l2_valid_lane;

  // SIMD Lane 3
  integer l3_addr00, l3_addr10, l3_addr01, l3_addr11;
  integer l3_exp_addr;
  integer l3_mem00, l3_mem10, l3_mem01, l3_mem11;
  integer l3_valid_lane;

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
  // Función auxiliar para dirección lineal (solo para logs)
  // --------------------------------------------------------------------------
  function automatic int linaddr_tb(
    input int x,
    input int y,
    input int width
  );
  begin
    linaddr_tb = (y * width) + x;
  end
  endfunction

  // --------------------------------------------------------------------------
  // Task: volcar mem_out a archivo HEX
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

        if (addr < MEM_DEPTH)
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
  // Task: dump parcial de bancos de entrada
  // --------------------------------------------------------------------------
  task automatic dump_input_banks(
    input string tag,
    input int    N
  );
    int i;
  begin
    $display("\n[TB][%s] Dump inicial de bancos de entrada (primeros %0d elementos):", tag, N);

    for (i = 0; i < N; i = i + 1)
      $display("[TB][%s] mem_in [%0d]  = 0x%02h",  tag, i, dut.mem_in.mem[i]);

    for (i = 0; i < N; i = i + 1)
      $display("[TB][%s] mem_in1[%0d] = 0x%02h",  tag, i, dut.mem_in1.mem[i]);

    for (i = 0; i < N; i = i + 1)
      $display("[TB][%s] mem_in2[%0d] = 0x%02h",  tag, i, dut.mem_in2.mem[i]);

    for (i = 0; i < N; i = i + 1)
      $display("[TB][%s] mem_in3[%0d] = 0x%02h",  tag, i, dut.mem_in3.mem[i]);
  end
  endtask

  // --------------------------------------------------------------------------
  // Secuencia principal: reset, SEQ, reset, SIMD4
  // --------------------------------------------------------------------------
  initial begin
    integer fd_check;
    integer i;
    int     mismatches;
    int     limit_pixels;

    rst_n        = 1'b0;
    start_sw     = 1'b0;
    mode_simd_sw = 1'b0;
    run_id       = 0;

    // Inicializar golden_seq a 0x00
    for (i = 0; i < MEM_DEPTH; i = i + 1)
      golden_seq[i] = 8'h00;

    // Reset inicial
    #100_000;              // 100 ns
    rst_n = 1'b1;

    // Esperar a fin de limpieza de mem_out del top
    @(negedge dut.clear_active);
    repeat (5) @(posedge clk_50);

    // Forzar parámetros IN_W_CFG x IN_H_CFG, escala SCALE_Q88_CFG Q8.8
    force dut.in_w_cfg      = IN_W_CFG;
    force dut.in_h_cfg      = IN_H_CFG;
    force dut.scale_q88_cfg = SCALE_Q88_CFG;
    $display("[TB] Forzando in_w_cfg=%0d, in_h_cfg=%0d, scale_q88_cfg=%0d (Q8.8)",
             IN_W_CFG, IN_H_CFG, SCALE_Q88_CFG);

    // Cargar imagen de entrada en mem_in
    fd_check = $fopen(IN_HEX_FILE, "r");
    if (fd_check == 0) begin
      $display("[TB][ERROR] No se pudo abrir '%s' para lectura.", IN_HEX_FILE);
    end else begin
      $fclose(fd_check);
      $display("[TB] Cargando imagen de entrada desde '%s' en mem_in...", IN_HEX_FILE);
      $readmemh(IN_HEX_FILE, dut.mem_in.mem);

      for (i = 0; i < 16; i = i + 1)
        $display("[TB] mem_in[%0d] = 0x%02h", i, dut.mem_in.mem[i]);

      // Copiar a bancos SIMD
      for (i = 0; i < MEM_DEPTH; i = i + 1) begin
        dut.mem_in1.mem[i] = dut.mem_in.mem[i];
        dut.mem_in2.mem[i] = dut.mem_in.mem[i];
        dut.mem_in3.mem[i] = dut.mem_in.mem[i];
      end
    end

    dump_input_banks("ANTES_SEQ", 16);

    // ======================================================
    // CORRIDA 1: Núcleo SECUENCIAL
    // ======================================================
    run_id       = 1;
    mode_simd_sw = 1'b0;

    $display("\n[TB] ===== INICIO CORRIDA 1: SECUENCIAL (%0dx%0d) ===== t=%0t\n",
             IN_W_CFG, IN_H_CFG, $time);

    $display("[TB][SEQ] Forzando start_pulse_sw en t=%0t", $time);
    force dut.start_pulse_sw = 1'b1;
    @(posedge clk_50);
    force dut.start_pulse_sw = 1'b0;
    @(posedge clk_50);
    release dut.start_pulse_sw;

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

    // Guardar salida secuencial como golden_seq
    for (i = 0; i < MEM_DEPTH; i = i + 1)
      golden_seq[i] = dut.mem_out.mem[i];

    $display("[TB][SEQ] Golden sequence almacenada en golden_seq[0..%0d]", MEM_DEPTH-1);

    // ======================================================
    // Reset entre corridas
    // ======================================================
    $display("\n[TB] ===== RESETEANDO PARA CORRIDA 2: SIMD4 (%0dx%0d) ===== t=%0t\n",
             IN_W_CFG, IN_H_CFG, $time);

    rst_n = 1'b0;
    #100_000;
    rst_n = 1'b1;

    @(negedge dut.clear_active);
    repeat (5) @(posedge clk_50);

    // Forzar de nuevo parámetros de imagen/escala
    force dut.in_w_cfg      = IN_W_CFG;
    force dut.in_h_cfg      = IN_H_CFG;
    force dut.scale_q88_cfg = SCALE_Q88_CFG;
    $display("[TB] (Corrida 2) Forzando in_w_cfg=%0d, in_h_cfg=%0d, scale_q88_cfg=%0d (Q8.8)",
             IN_W_CFG, IN_H_CFG, SCALE_Q88_CFG);

    dump_input_banks("ANTES_SIMD", 16);

    // ======================================================
    // CORRIDA 2: Núcleo SIMD4
    // ======================================================
    run_id       = 2;
    mode_simd_sw = 1'b1;

    $display("\n[TB] ===== INICIO CORRIDA 2: SIMD4 (%0dx%0d) ===== t=%0t\n",
             IN_W_CFG, IN_H_CFG, $time);

    $display("[TB][SIMD4] Forzando start_pulse_sw en t=%0t", $time);
    force dut.start_pulse_sw = 1'b1;
    @(posedge clk_50);
    force dut.start_pulse_sw = 1'b0;
    @(posedge clk_50);
    release dut.start_pulse_sw;

    @(posedge dut.done);

    $display("[TB][SIMD4] DONE detectado en t=%0t", $time);
    $display("[TB][SIMD4] mode_simd_eff = 1 (0=SEQ,1=SIMD4)", dut.mode_simd_eff);

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

    // ------------------------------------------------------------------
    // Comparación final SEQ vs SIMD4
    // ------------------------------------------------------------------
    mismatches   = 0;
    limit_pixels = ow * oh;

    if (limit_pixels > MEM_DEPTH)
      limit_pixels = MEM_DEPTH;

    $display("\n[TB][CMP] Comparando golden_seq (SEQ) vs mem_out (SIMD4) en %0d píxeles...", limit_pixels);

    for (i = 0; i < limit_pixels; i = i + 1) begin
      if (golden_seq[i] !== dut.mem_out.mem[i]) begin
        mismatches++;
        $display("[CMP][%0d] addr=%0d SEQ=0x%02h SIMD=0x%02h",
                 mismatches, i, golden_seq[i], dut.mem_out.mem[i]);

        if (mismatches >= DBG_SIMD_MAX_MISM) begin
          $display("[CMP] Alcanzado límite de mismatches a mostrar (%0d).", DBG_SIMD_MAX_MISM);
          break;
        end
      end
    end

    $display("[CMP] Total mismatches=%0d (de %0d píxeles revisados).", mismatches, limit_pixels);
    if (mismatches == 0)
      $display("[CMP] ¡Perfecto! SEQ y SIMD4 coinciden en todos los píxeles revisados.");

    $display("[TB] Simulación terminada correctamente.");
    $finish;
  end

  // --------------------------------------------------------------------------
  // Watchdog
  // --------------------------------------------------------------------------
  initial begin
    cycles     = 0;
    MAX_CYCLES = 10_000_000;
    forever begin
      @(posedge clk_50);
      cycles++;
      if (cycles == 1_000_000)
        $display("[TB][INFO] 1M ciclos...");
      if (cycles > MAX_CYCLES)
        $fatal(1, "[TB][TIMEOUT] No se recibió 'done' tras %0d ciclos.", cycles);
    end
  end

  // --------------------------------------------------------------------------
  // Monitor SECUENCIAL (detallado, solo addr<16)
  // --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    if (DBG_SEQ_MONITOR &&
        !dut.mode_simd_eff &&
        dut.u_core_seq.out_we &&
        (dut.u_core_seq.out_waddr < 16)) begin

      seq_addr00 = linaddr_tb(dut.u_core_seq.xi_base,
                              dut.u_core_seq.yi_base,
                              dut.in_w);
      seq_addr10 = linaddr_tb(dut.u_core_seq.xi_base + 1,
                              dut.u_core_seq.yi_base,
                              dut.in_w);
      seq_addr01 = linaddr_tb(dut.u_core_seq.xi_base,
                              dut.u_core_seq.yi_base + 1,
                              dut.in_w);
      seq_addr11 = linaddr_tb(dut.u_core_seq.xi_base + 1,
                              dut.u_core_seq.yi_base + 1,
                              dut.in_w);

      seq_exp_addr = dut.u_core_seq.oy_cur * dut.out_w_s_seq +
                     dut.u_core_seq.ox_cur;

      seq_mem00 = dut.mem_in.mem[seq_addr00];
      seq_mem10 = dut.mem_in.mem[seq_addr10];
      seq_mem01 = dut.mem_in.mem[seq_addr01];
      seq_mem11 = dut.mem_in.mem[seq_addr11];

      $display("[SEQ][RUN=%0d][t=%0t] WRITE ox=%0d oy=%0d addr=%0d pix=0x%02h | sx_fix=%0d.%0d sy_fix=%0d.%0d | xi=%0d yi=%0d fx_q=%0d fy_q=%0d | addr00=%0d addr10=%0d addr01=%0d addr11=%0d | I00=%0d I10=%0d I01=%0d I11=%0d",
        run_id,
        $time,
        dut.u_core_seq.ox_cur,
        dut.u_core_seq.oy_cur,
        dut.u_core_seq.out_waddr,
        dut.u_core_seq.out_wdata,
        dut.u_core_seq.sx_fix[23:8], dut.u_core_seq.sx_fix[7:0],
        dut.u_core_seq.sy_fix[23:8], dut.u_core_seq.sy_fix[7:0],
        dut.u_core_seq.xi_base,
        dut.u_core_seq.yi_base,
        dut.u_core_seq.fx_q,
        dut.u_core_seq.fy_q,
        seq_addr00, seq_addr10, seq_addr01, seq_addr11,
        dut.u_core_seq.I00,
        dut.u_core_seq.I10,
        dut.u_core_seq.I01,
        dut.u_core_seq.I11
      );

      $display("[SEQ][ADDRCHK] t=%0t ox=%0d oy=%0d out_w=%0d | exp_addr=%0d out_waddr=%0d",
        $time,
        dut.u_core_seq.ox_cur,
        dut.u_core_seq.oy_cur,
        dut.out_w_s_seq,
        seq_exp_addr,
        dut.u_core_seq.out_waddr
      );

      $display("[SEQ][MEMCHK]  t=%0t addr00=%0d I00=%0d mem_in=%0d | addr10=%0d I10=%0d mem_in=%0d | addr01=%0d I01=%0d mem_in=%0d | addr11=%0d I11=%0d mem_in=%0d",
        $time,
        seq_addr00, dut.u_core_seq.I00, seq_mem00,
        seq_addr10, dut.u_core_seq.I10, seq_mem10,
        seq_addr01, dut.u_core_seq.I01, seq_mem01,
        seq_addr11, dut.u_core_seq.I11, seq_mem11
      );
    end
  end

  // --------------------------------------------------------------------------
  // Monitor SIMD4
  // --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    if (dut.mode_simd_eff) begin

      // Resumen por ciclo
      if (DBG_SIMD_SUMMARY &&
          (dut.u_core_simd4.out_we0 || dut.u_core_simd4.out_we1 ||
           dut.u_core_simd4.out_we2 || dut.u_core_simd4.out_we3)) begin

        if (dut.u_core_simd4.out_we0)
          $display("[SIMD][RUN=%0d][t=%0t] L0 WRITE addr=%0d pix=0x%02h",
            run_id, $time, dut.u_core_simd4.out_waddr0, dut.u_core_simd4.out_wdata0);

        if (dut.u_core_simd4.out_we1)
          $display("[SIMD][RUN=%0d][t=%0t] L1 WRITE addr=%0d pix=0x%02h",
            run_id, $time, dut.u_core_simd4.out_waddr1, dut.u_core_simd4.out_wdata1);

        if (dut.u_core_simd4.out_we2)
          $display("[SIMD][RUN=%0d][t=%0t] L2 WRITE addr=%0d pix=0x%02h",
            run_id, $time, dut.u_core_simd4.out_waddr2, dut.u_core_simd4.out_wdata2);

        if (dut.u_core_simd4.out_we3)
          $display("[SIMD][RUN=%0d][t=%0t] L3 WRITE addr=%0d pix=0x%02h",
            run_id, $time, dut.u_core_simd4.out_waddr3, dut.u_core_simd4.out_wdata3);
      end

      if (DBG_SIMD_DETAILED) begin
        // -------- Lane 0 --------
        if (dut.u_core_simd4.out_we0 && (dut.u_core_simd4.out_waddr0 < 16)) begin
          l0_addr00 = linaddr_tb(dut.u_core_simd4.xi_base_lane[0],
                                 dut.u_core_simd4.yi_base_row,
                                 dut.in_w);
          l0_addr10 = linaddr_tb(dut.u_core_simd4.xi_base_lane[0] + 1,
                                 dut.u_core_simd4.yi_base_row,
                                 dut.in_w);
          l0_addr01 = linaddr_tb(dut.u_core_simd4.xi_base_lane[0],
                                 dut.u_core_simd4.yi_base_row + 1,
                                 dut.in_w);
          l0_addr11 = linaddr_tb(dut.u_core_simd4.xi_base_lane[0] + 1,
                                 dut.u_core_simd4.yi_base_row + 1,
                                 dut.in_w);

          l0_exp_addr  = dut.u_core_simd4.oy_cur * dut.out_w_s_simd +
                         dut.u_core_simd4.ox_lane[0];
          l0_valid_lane = (dut.u_core_simd4.ox_lane[0] < dut.out_w_s_simd) ? 1 : 0;

          l0_mem00 = dut.mem_in.mem[l0_addr00];
          l0_mem10 = dut.mem_in.mem[l0_addr10];
          l0_mem01 = dut.mem_in.mem[l0_addr01];
          l0_mem11 = dut.mem_in.mem[l0_addr11];

          $display("[SIMD] L0 WRITE ox=%0d oy=%0d addr=%0d pix=0x%0h | sx_int=%0d ax_q=%0d sy_int=%0d ay_q=%0d xi=%0d yi=%0d fx_q=%0d fy_q=%0d | addr00=%0d addr10=%0d addr01=%0d addr11=%0d | I00=%0d I10=%0d I01=%0d I11=%0d",
            dut.u_core_simd4.ox_lane[0],
            dut.u_core_simd4.oy_cur,
            dut.u_core_simd4.out_waddr0,
            dut.u_core_simd4.out_wdata0,
            // Log geométrico aproximado: se usa xi_base y fx_q como "sx_int / ax_q"
            dut.u_core_simd4.xi_base_lane[0],
            dut.u_core_simd4.fx_q_lane[0],
            dut.u_core_simd4.sy_int_row,
            dut.u_core_simd4.ay_q_row,
            dut.u_core_simd4.xi_base_lane[0],
            dut.u_core_simd4.yi_base_row,
            dut.u_core_simd4.fx_q_lane[0],
            dut.u_core_simd4.fy_q_row,
            l0_addr00, l0_addr10, l0_addr01, l0_addr11,
            dut.u_core_simd4.I00[0],
            dut.u_core_simd4.I10[0],
            dut.u_core_simd4.I01[0],
            dut.u_core_simd4.I11[0]
          );

          $display("[SIMD] L0 ADDRCHK t=%0t ox=%0d oy=%0d out_w=%0d | exp_addr=%0d out_waddr=%0d | valid_lane=%0d",
            $time,
            dut.u_core_simd4.ox_lane[0],
            dut.u_core_simd4.oy_cur,
            dut.out_w_s_simd,
            l0_exp_addr,
            dut.u_core_simd4.out_waddr0,
            l0_valid_lane
          );

          $display("[SIMD] L0 MEMCHK  t=%0t addr00=%0d I00=%0d mem_in=%0d | addr10=%0d I10=%0d mem_in=%0d | addr01=%0d I01=%0d mem_in=%0d | addr11=%0d I11=%0d mem_in=%0d",
            $time,
            l0_addr00, dut.u_core_simd4.I00[0], l0_mem00,
            l0_addr10, dut.u_core_simd4.I10[0], l0_mem10,
            l0_addr01, dut.u_core_simd4.I01[0], l0_mem01,
            l0_addr11, dut.u_core_simd4.I11[0], l0_mem11
          );
        end

        // -------- Lane 1 --------
        if (dut.u_core_simd4.out_we1 && (dut.u_core_simd4.out_waddr1 < 16)) begin
          l1_addr00 = linaddr_tb(dut.u_core_simd4.xi_base_lane[1],
                                 dut.u_core_simd4.yi_base_row,
                                 dut.in_w);
          l1_addr10 = linaddr_tb(dut.u_core_simd4.xi_base_lane[1] + 1,
                                 dut.u_core_simd4.yi_base_row,
                                 dut.in_w);
          l1_addr01 = linaddr_tb(dut.u_core_simd4.xi_base_lane[1],
                                 dut.u_core_simd4.yi_base_row + 1,
                                 dut.in_w);
          l1_addr11 = linaddr_tb(dut.u_core_simd4.xi_base_lane[1] + 1,
                                 dut.u_core_simd4.yi_base_row + 1,
                                 dut.in_w);

          l1_exp_addr  = dut.u_core_simd4.oy_cur * dut.out_w_s_simd +
                         dut.u_core_simd4.ox_lane[1];
          l1_valid_lane = (dut.u_core_simd4.ox_lane[1] < dut.out_w_s_simd) ? 1 : 0;

          l1_mem00 = dut.mem_in1.mem[l1_addr00];
          l1_mem10 = dut.mem_in1.mem[l1_addr10];
          l1_mem01 = dut.mem_in1.mem[l1_addr01];
          l1_mem11 = dut.mem_in1.mem[l1_addr11];

          $display("[SIMD] L1 WRITE ox=%0d oy=%0d addr=%0d pix=0x%0h | sx_int=%0d ax_q=%0d sy_int=%0d ay_q=%0d xi=%0d yi=%0d fx_q=%0d fy_q=%0d | addr00=%0d addr10=%0d addr01=%0d addr11=%0d | I00=%0d I10=%0d I01=%0d I11=%0d",
            dut.u_core_simd4.ox_lane[1],
            dut.u_core_simd4.oy_cur,
            dut.u_core_simd4.out_waddr1,
            dut.u_core_simd4.out_wdata1,
            dut.u_core_simd4.xi_base_lane[1],
            dut.u_core_simd4.fx_q_lane[1],
            dut.u_core_simd4.sy_int_row,
            dut.u_core_simd4.ay_q_row,
            dut.u_core_simd4.xi_base_lane[1],
            dut.u_core_simd4.yi_base_row,
            dut.u_core_simd4.fx_q_lane[1],
            dut.u_core_simd4.fy_q_row,
            l1_addr00, l1_addr10, l1_addr01, l1_addr11,
            dut.u_core_simd4.I00[1],
            dut.u_core_simd4.I10[1],
            dut.u_core_simd4.I01[1],
            dut.u_core_simd4.I11[1]
          );

          $display("[SIMD] L1 ADDRCHK t=%0t ox=%0d oy=%0d out_w=%0d | exp_addr=%0d out_waddr=%0d | valid_lane=%0d",
            $time,
            dut.u_core_simd4.ox_lane[1],
            dut.u_core_simd4.oy_cur,
            dut.out_w_s_simd,
            l1_exp_addr,
            dut.u_core_simd4.out_waddr1,
            l1_valid_lane
          );

          $display("[SIMD] L1 MEMCHK  t=%0t addr00=%0d I00=%0d mem_in=%0d | addr10=%0d I10=%0d mem_in=%0d | addr01=%0d I01=%0d mem_in=%0d | addr11=%0d I11=%0d mem_in=%0d",
            $time,
            l1_addr00, dut.u_core_simd4.I00[1], l1_mem00,
            l1_addr10, dut.u_core_simd4.I10[1], l1_mem10,
            l1_addr01, dut.u_core_simd4.I01[1], l1_mem01,
            l1_addr11, dut.u_core_simd4.I11[1], l1_mem11
          );
        end

        // -------- Lane 2 --------
        if (dut.u_core_simd4.out_we2 && (dut.u_core_simd4.out_waddr2 < 16)) begin
          l2_addr00 = linaddr_tb(dut.u_core_simd4.xi_base_lane[2],
                                 dut.u_core_simd4.yi_base_row,
                                 dut.in_w);
          l2_addr10 = linaddr_tb(dut.u_core_simd4.xi_base_lane[2] + 1,
                                 dut.u_core_simd4.yi_base_row,
                                 dut.in_w);
          l2_addr01 = linaddr_tb(dut.u_core_simd4.xi_base_lane[2],
                                 dut.u_core_simd4.yi_base_row + 1,
                                 dut.in_w);
          l2_addr11 = linaddr_tb(dut.u_core_simd4.xi_base_lane[2] + 1,
                                 dut.u_core_simd4.yi_base_row + 1,
                                 dut.in_w);

          l2_exp_addr  = dut.u_core_simd4.oy_cur * dut.out_w_s_simd +
                         dut.u_core_simd4.ox_lane[2];
          l2_valid_lane = (dut.u_core_simd4.ox_lane[2] < dut.out_w_s_simd) ? 1 : 0;

          l2_mem00 = dut.mem_in2.mem[l2_addr00];
          l2_mem10 = dut.mem_in2.mem[l2_addr10];
          l2_mem01 = dut.mem_in2.mem[l2_addr01];
          l2_mem11 = dut.mem_in2.mem[l2_addr11];

          $display("[SIMD] L2 WRITE ox=%0d oy=%0d addr=%0d pix=0x%0h | sx_int=%0d ax_q=%0d sy_int=%0d ay_q=%0d xi=%0d yi=%0d fx_q=%0d fy_q=%0d | addr00=%0d addr10=%0d addr01=%0d addr11=%0d | I00=%0d I10=%0d I01=%0d I11=%0d",
            dut.u_core_simd4.ox_lane[2],
            dut.u_core_simd4.oy_cur,
            dut.u_core_simd4.out_waddr2,
            dut.u_core_simd4.out_wdata2,
            dut.u_core_simd4.xi_base_lane[2],
            dut.u_core_simd4.fx_q_lane[2],
            dut.u_core_simd4.sy_int_row,
            dut.u_core_simd4.ay_q_row,
            dut.u_core_simd4.xi_base_lane[2],
            dut.u_core_simd4.yi_base_row,
            dut.u_core_simd4.fx_q_lane[2],
            dut.u_core_simd4.fy_q_row,
            l2_addr00, l2_addr10, l2_addr01, l2_addr11,
            dut.u_core_simd4.I00[2],
            dut.u_core_simd4.I10[2],
            dut.u_core_simd4.I01[2],
            dut.u_core_simd4.I11[2]
          );

          $display("[SIMD] L2 ADDRCHK t=%0t ox=%0d oy=%0d out_w=%0d | exp_addr=%0d out_waddr=%0d | valid_lane=%0d",
            $time,
            dut.u_core_simd4.ox_lane[2],
            dut.u_core_simd4.oy_cur,
            dut.out_w_s_simd,
            l2_exp_addr,
            dut.u_core_simd4.out_waddr2,
            l2_valid_lane
          );

          $display("[SIMD] L2 MEMCHK  t=%0t addr00=%0d I00=%0d mem_in=%0d | addr10=%0d I10=%0d mem_in=%0d | addr01=%0d I01=%0d mem_in=%0d | addr11=%0d I11=%0d mem_in=%0d",
            $time,
            l2_addr00, dut.u_core_simd4.I00[2], l2_mem00,
            l2_addr10, dut.u_core_simd4.I10[2], l2_mem10,
            l2_addr01, dut.u_core_simd4.I01[2], l2_mem01,
            l2_addr11, dut.u_core_simd4.I11[2], l2_mem11
          );
        end

        // -------- Lane 3 --------
        if (dut.u_core_simd4.out_we3 && (dut.u_core_simd4.out_waddr3 < 16)) begin
          l3_addr00 = linaddr_tb(dut.u_core_simd4.xi_base_lane[3],
                                 dut.u_core_simd4.yi_base_row,
                                 dut.in_w);
          l3_addr10 = linaddr_tb(dut.u_core_simd4.xi_base_lane[3] + 1,
                                 dut.u_core_simd4.yi_base_row,
                                 dut.in_w);
          l3_addr01 = linaddr_tb(dut.u_core_simd4.xi_base_lane[3],
                                 dut.u_core_simd4.yi_base_row + 1,
                                 dut.in_w);
          l3_addr11 = linaddr_tb(dut.u_core_simd4.xi_base_lane[3] + 1,
                                 dut.u_core_simd4.yi_base_row + 1,
                                 dut.in_w);

          l3_exp_addr  = dut.u_core_simd4.oy_cur * dut.out_w_s_simd +
                         dut.u_core_simd4.ox_lane[3];
          l3_valid_lane = (dut.u_core_simd4.ox_lane[3] < dut.out_w_s_simd) ? 1 : 0;

          l3_mem00 = dut.mem_in3.mem[l3_addr00];
          l3_mem10 = dut.mem_in3.mem[l3_addr10];
          l3_mem01 = dut.mem_in3.mem[l3_addr01];
          l3_mem11 = dut.mem_in3.mem[l3_addr11];

          $display("[SIMD] L3 WRITE ox=%0d oy=%0d addr=%0d pix=0x%0h | sx_int=%0d ax_q=%0d sy_int=%0d ay_q=%0d xi=%0d yi=%0d fx_q=%0d fy_q=%0d | addr00=%0d addr10=%0d addr01=%0d addr11=%0d | I00=%0d I10=%0d I01=%0d I11=%0d",
            dut.u_core_simd4.ox_lane[3],
            dut.u_core_simd4.oy_cur,
            dut.u_core_simd4.out_waddr3,
            dut.u_core_simd4.out_wdata3,
            dut.u_core_simd4.xi_base_lane[3],
            dut.u_core_simd4.fx_q_lane[3],
            dut.u_core_simd4.sy_int_row,
            dut.u_core_simd4.ay_q_row,
            dut.u_core_simd4.xi_base_lane[3],
            dut.u_core_simd4.yi_base_row,
            dut.u_core_simd4.fx_q_lane[3],
            dut.u_core_simd4.fy_q_row,
            l3_addr00, l3_addr10, l3_addr01, l3_addr11,
            dut.u_core_simd4.I00[3],
            dut.u_core_simd4.I10[3],
            dut.u_core_simd4.I01[3],
            dut.u_core_simd4.I11[3]
          );

          $display("[SIMD] L3 ADDRCHK t=%0t ox=%0d oy=%0d out_w=%0d | exp_addr=%0d out_waddr=%0d | valid_lane=%0d",
            $time,
            dut.u_core_simd4.ox_lane[3],
            dut.u_core_simd4.oy_cur,
            dut.out_w_s_simd,
            l3_exp_addr,
            dut.u_core_simd4.out_waddr3,
            l3_valid_lane
          );

          $display("[SIMD] L3 MEMCHK  t=%0t addr00=%0d I00=%0d mem_in=%0d | addr10=%0d I10=%0d mem_in=%0d | addr01=%0d I01=%0d mem_in=%0d | addr11=%0d I11=%0d mem_in=%0d",
            $time,
            l3_addr00, dut.u_core_simd4.I00[3], l3_mem00,
            l3_addr10, dut.u_core_simd4.I10[3], l3_mem10,
            l3_addr01, dut.u_core_simd4.I01[3], l3_mem01,
            l3_addr11, dut.u_core_simd4.I11[3], l3_mem11
          );
        end
      end
    end
  end

endmodule
