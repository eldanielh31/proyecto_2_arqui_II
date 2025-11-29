`timescale 1ps/1ps

// ============================================================================
// tb_bilinear_seq — Testbench para top_dsa_wide (versión sin JTAG / sin mem_out)
//   - Ejecuta primero el núcleo SECUENCIAL (mode_simd_sw = 0).
//   - Luego hace reset y ejecuta el núcleo SIMD4 (mode_simd_sw = 1).
//   - Genera clock, reset y pulso de start.
//   - Carga la imagen de entrada desde archivo HEX en formato wide (32-bit).
//   - Captura las escrituras de salida directamente de los cores:
//       * u_core_seq: out_waddr_seq / out_wdata_seq / out_we_seq
//       * u_core_simd4: out_waddr_simd[0..3] / out_wdata_simd[0..3] / out_we_simd[0..3]
//   - Vuelca la salida SEQ y SIMD a archivos HEX.
//   - Compara SEQ (golden) vs SIMD4.
// ============================================================================

module tb_bilinear_seq;

  // --------------------------------------------------------------------------
  // Parámetros
  // --------------------------------------------------------------------------
  localparam int    AW             = 18;          // ancho de dirección para entrada
  localparam int    MEM_DEPTH      = (1 << AW);   // profundidad en palabras (32 bits)
  localparam time   T_CLK_PS       = 20_000;      // 20 ns -> 50 MHz

  // Configuración de imagen / escala
  // localparam int    IN_W_CFG       = 16;          // ancho de la imagen de entrada
  // localparam int    IN_H_CFG       = 16;          // alto de la imagen de entrada
  // localparam int    CFG_SCALE_Q88  = 16'd205;     // (no se usa directamente en el TB nuevo)

  // localparam string IN_HEX_FILE    = "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_16x16.hex";

  localparam int    IN_W_CFG       = 512;          // ancho de la imagen de entrada
  localparam int    IN_H_CFG       = 512;          // alto de la imagen de entrada
  localparam int    CFG_SCALE_Q88  = 16'd205;     // (no se usa directamente en el TB nuevo)

  localparam string IN_HEX_FILE    = "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_in_512x512.hex";

  // Flags de debug
  localparam bit DBG_SEQ_MONITOR      = 1'b1;
  localparam bit DBG_SIMD_SUMMARY     = 1'b1;
  localparam bit DBG_SIMD_DETAILED    = 1'b0;   // en esta versión se deja en 0
  localparam int DBG_SIMD_MAX_MISM    = 50;

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
  // Variables para watchdog y dimensiones
  // --------------------------------------------------------------------------
  longint unsigned cycles;
  longint unsigned MAX_CYCLES;

  int ow;
  int oh;
  int run_id;  // 0=none, 1=seq, 2=simd

  // --------------------------------------------------------------------------
  // Buffers de salida (capturados en el TB)
  //   - Máximo: MEM_DEPTH * 4 píxeles (4 bytes por word)
  // --------------------------------------------------------------------------
  localparam int OUT_PIX_MAX = MEM_DEPTH * 4;

  logic [7:0] golden_seq      [0:OUT_PIX_MAX-1];  // golden de SEQ
  logic [7:0] out_seq_pixels  [0:OUT_PIX_MAX-1];  // capturado de core secuencial
  logic [7:0] out_simd_pixels [0:OUT_PIX_MAX-1];  // capturado de core SIMD

  // Helper array para cargar imagen de entrada (formato byte)
  logic [7:0] byte_image_in [0:(IN_W_CFG*IN_H_CFG)-1];

  // --------------------------------------------------------------------------
  // DUT
  // --------------------------------------------------------------------------
  top_dsa_seq #(
    .IMG_W         (IN_W_CFG),
    .IMG_H         (IN_H_CFG),
    .AW            (AW),
    .SIMULATION    (1),
    .DEB_W         (20),
    .RST_STRETCH_W (22)
  ) dut (
    .clk_50        (clk_50),
    .rst_n         (rst_n),

    .start_sw      (start_sw),
    .mode_simd_sw  (mode_simd_sw),

    .led_done      (led_done),
    .led_reset_evt (led_reset_evt),
    .led_start_on  (led_start_on),
    .led_simd_mode (led_simd_mode)
  );

  // --------------------------------------------------------------------------
  // Clock 50 MHz
  // --------------------------------------------------------------------------
  initial begin
    clk_50 = 1'b0;
    forever #(T_CLK_PS/2) clk_50 = ~clk_50;
  end

  // --------------------------------------------------------------------------
  // Función auxiliar para dirección lineal (por si se ocupa)
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
  // Task: volcar píxeles de un buffer a archivo HEX (byte por línea)
  // --------------------------------------------------------------------------
  task automatic dump_pixels_seq(
    input string fname,
    input int    ow_local,
    input int    oh_local
  );
    integer fd;
    int x, y;
    int pixel_addr;
    reg [7:0] val;
  begin
    fd = $fopen(fname, "w");
    if (fd == 0) begin
      $display("[TB][ERROR] No se pudo abrir '%s' para escritura.", fname);
      disable dump_pixels_seq;
    end

    $display("[TB][SEQ] Volcando salida SEQ (%0dx%0d) a '%s'", ow_local, oh_local, fname);

    for (y = 0; y < oh_local; y = y + 1) begin
      for (x = 0; x < ow_local; x = x + 1) begin
        pixel_addr = y * ow_local + x;
        if (pixel_addr < OUT_PIX_MAX)
          val = out_seq_pixels[pixel_addr];
        else
          val = 8'h00;
        $fdisplay(fd, "%02h", val);
      end
    end

    $fclose(fd);
    $display("[TB][SEQ] Dump completado.");
  end
  endtask

  task automatic dump_pixels_simd(
    input string fname,
    input int    ow_local,
    input int    oh_local
  );
    integer fd;
    int x, y;
    int pixel_addr;
    reg [7:0] val;
  begin
    fd = $fopen(fname, "w");
    if (fd == 0) begin
      $display("[TB][ERROR] No se pudo abrir '%s' para escritura.", fname);
      disable dump_pixels_simd;
    end

    $display("[TB][SIMD] Volcando salida SIMD4 (%0dx%0d) a '%s'", ow_local, oh_local, fname);

    for (y = 0; y < oh_local; y = y + 1) begin
      for (x = 0; x < ow_local; x = x + 1) begin
        pixel_addr = y * ow_local + x;
        if (pixel_addr < OUT_PIX_MAX)
          val = out_simd_pixels[pixel_addr];
        else
          val = 8'h00;
        $fdisplay(fd, "%02h", val);
      end
    end

    $fclose(fd);
    $display("[TB][SIMD] Dump completado.");
  end
  endtask

  // --------------------------------------------------------------------------
  // Task: Load byte-format HEX into wide memory de entrada (mem_in)
  // --------------------------------------------------------------------------
  task automatic load_image_to_wide_mem(
    input string hex_file
  );
    integer fd_check;
    integer i, word_idx;
    logic [31:0] temp_word;
  begin
    fd_check = $fopen(hex_file, "r");
    if (fd_check == 0) begin
      $display("[TB][ERROR] No se pudo abrir '%s' para lectura.", hex_file);
    end else begin
      $fclose(fd_check);
      
      // Load into byte array first
      $display("[TB] Cargando imagen desde '%s'...", hex_file);
      $readmemh(hex_file, byte_image_in);
      
      // Convert byte array to wide memory format (4 píxeles/word)
      for (i = 0; i < (IN_W_CFG * IN_H_CFG); i = i + 4) begin
        word_idx = i >> 2;
        
        temp_word[7:0]   = (i+0 < IN_W_CFG*IN_H_CFG) ? byte_image_in[i+0] : 8'h00;
        temp_word[15:8]  = (i+1 < IN_W_CFG*IN_H_CFG) ? byte_image_in[i+1] : 8'h00;
        temp_word[23:16] = (i+2 < IN_W_CFG*IN_H_CFG) ? byte_image_in[i+2] : 8'h00;
        temp_word[31:24] = (i+3 < IN_W_CFG*IN_H_CFG) ? byte_image_in[i+3] : 8'h00;
        
        dut.mem_in.mem[word_idx] = temp_word;
      end
      
      $display("[TB] Imagen cargada en mem_in. Primeros 4 words (16 bytes):");
      for (i = 0; i < 4; i = i + 1) begin
        $display("[TB] mem_in[%0d] = 0x%08h [px0=%02h px1=%02h px2=%02h px3=%02h]",
                 i, dut.mem_in.mem[i],
                 dut.mem_in.mem[i][7:0],
                 dut.mem_in.mem[i][15:8],
                 dut.mem_in.mem[i][23:16],
                 dut.mem_in.mem[i][31:24]);
      end
    end
  end
  endtask

  // --------------------------------------------------------------------------
  // Task: dump primeros N words de memoria de entrada
  // --------------------------------------------------------------------------
  task automatic dump_input_mem(
    input string tag,
    input int    N
  );
    int i;
  begin
    $display("\n[TB][%s] Dump de memoria de entrada (primeros %0d words):", tag, N);
    for (i = 0; i < N; i = i + 1) begin
      $display("[TB][%s] mem_in[%0d] = 0x%08h", tag, i, dut.mem_in.mem[i]);
    end
  end
  endtask

  // --------------------------------------------------------------------------
  // Captura de escrituras de salida desde los cores
  //   - Se almacenan en out_seq_pixels y out_simd_pixels
  // --------------------------------------------------------------------------
  integer idx_addr;

  initial begin
    int i;
    for (i = 0; i < OUT_PIX_MAX; i = i + 1) begin
      out_seq_pixels[i]  = 8'h00;
      out_simd_pixels[i] = 8'h00;
      golden_seq[i]      = 8'h00;
    end
  end

  always @(posedge clk_50) begin
    // Secuencial
    if (dut.out_we_seq) begin
      idx_addr = dut.out_waddr_seq;
      if (idx_addr >= 0 && idx_addr < OUT_PIX_MAX)
        out_seq_pixels[idx_addr] <= dut.out_wdata_seq;
    end

    // SIMD4: cuatro lanes
    if (dut.out_we_simd0) begin
      idx_addr = dut.out_waddr_simd0;
      if (idx_addr >= 0 && idx_addr < OUT_PIX_MAX)
        out_simd_pixels[idx_addr] <= dut.out_wdata_simd0;
    end
    if (dut.out_we_simd1) begin
      idx_addr = dut.out_waddr_simd1;
      if (idx_addr >= 0 && idx_addr < OUT_PIX_MAX)
        out_simd_pixels[idx_addr] <= dut.out_wdata_simd1;
    end
    if (dut.out_we_simd2) begin
      idx_addr = dut.out_waddr_simd2;
      if (idx_addr >= 0 && idx_addr < OUT_PIX_MAX)
        out_simd_pixels[idx_addr] <= dut.out_wdata_simd2;
    end
    if (dut.out_we_simd3) begin
      idx_addr = dut.out_waddr_simd3;
      if (idx_addr >= 0 && idx_addr < OUT_PIX_MAX)
        out_simd_pixels[idx_addr] <= dut.out_wdata_simd3;
    end
  end

  // --------------------------------------------------------------------------
  // Secuencia principal: reset, SEQ, reset, SIMD4
  // --------------------------------------------------------------------------
  initial begin
    integer i;
    int     mismatches;
    int     limit_pixels;

    rst_n        = 1'b0;
    start_sw     = 1'b0;
    mode_simd_sw = 1'b0;
    run_id       = 0;

    // Reset inicial
    #100_000;
    rst_n = 1'b1;

    // Esperar unos ciclos tras reset
    repeat (10) @(posedge clk_50);

    // Cargar imagen en mem_in
    load_image_to_wide_mem(IN_HEX_FILE);
    dump_input_mem("INICIAL", 4);

    // ======================================================
    // CORRIDA 1: Núcleo SECUENCIAL
    // ======================================================
    run_id       = 1;
    mode_simd_sw = 1'b0;

    $display("\n[TB] ===== INICIO CORRIDA 1: SECUENCIAL (%0dx%0d) ===== t=%0t\n",
             IN_W_CFG, IN_H_CFG, $time);

    start_sw = 1'b1;
    @(posedge clk_50);
    start_sw = 1'b0;

    @(posedge dut.done);

    $display("[TB][SEQ] DONE detectado en t=%0t", $time);
    ow = dut.u_core_seq.o_out_w;
    oh = dut.u_core_seq.o_out_h;

    $display("[TB][SEQ] out_w = %0d, out_h = %0d", ow, oh);
    $display("[TB][SEQ] perf: flops=%0d mem_rd=%0d mem_wr=%0d",
             dut.u_core_seq.o_flop_count,
             dut.u_core_seq.o_mem_rd_count,
             dut.u_core_seq.o_mem_wr_count);

    dump_pixels_seq(
      "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_out_seq_wide.hex",
      ow, oh
    );

    // Guardar golden (byte por byte desde buffer SEQ)
    for (i = 0; i < (ow * oh); i = i + 1) begin
      if (i < OUT_PIX_MAX)
        golden_seq[i] = out_seq_pixels[i];
      else
        golden_seq[i] = 8'h00;
    end

    $display("[TB][SEQ] Golden sequence guardada (%0d pixels)", ow*oh);

    // ======================================================
    // Reset entre corridas
    // ======================================================
    $display("\n[TB] ===== RESETEANDO PARA CORRIDA 2: SIMD4 ===== t=%0t\n", $time);

    rst_n = 1'b0;
    #100_000;
    rst_n = 1'b1;

    repeat (10) @(posedge clk_50);

    // Recargar imagen
    load_image_to_wide_mem(IN_HEX_FILE);
    dump_input_mem("ANTES_SIMD", 4);

    // ======================================================
    // CORRIDA 2: Núcleo SIMD4
    // ======================================================
    run_id       = 2;
    mode_simd_sw = 1'b1;

    $display("\n[TB] ===== INICIO CORRIDA 2: SIMD4 (%0dx%0d) ===== t=%0t\n",
             IN_W_CFG, IN_H_CFG, $time);

    start_sw = 1'b1;
    @(posedge clk_50);
    start_sw = 1'b0;

    @(posedge dut.done);

    $display("[TB][SIMD4] DONE detectado en t=%0t", $time);
    ow = dut.u_core_simd4.o_out_w;
    oh = dut.u_core_simd4.o_out_h;

    $display("[TB][SIMD4] out_w = %0d, out_h = %0d", ow, oh);
    $display("[TB][SIMD4] perf: flops=%0d mem_rd=%0d mem_wr=%0d",
             dut.u_core_simd4.o_flop_count,
             dut.u_core_simd4.o_mem_rd_count,
             dut.u_core_simd4.o_mem_wr_count);

    dump_pixels_simd(
      "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_out_simd_wide.hex",
      ow, oh
    );

    // ------------------------------------------------------------------
    // Comparación final SEQ vs SIMD4
    // ------------------------------------------------------------------
    mismatches   = 0;
    limit_pixels = ow * oh;

    $display("\n[TB][CMP] Comparando golden_seq (SEQ) vs out_simd_pixels (SIMD4) en %0d píxeles...", 
             limit_pixels);

    for (i = 0; i < limit_pixels; i = i + 1) begin
      automatic logic [7:0] simd_val;
      if (i < OUT_PIX_MAX)
        simd_val = out_simd_pixels[i];
      else
        simd_val = 8'h00;
      
      if (golden_seq[i] !== simd_val) begin
        mismatches++;
        $display("[CMP][%0d] pixel=%0d SEQ=0x%02h SIMD=0x%02h",
                 mismatches, i, golden_seq[i], simd_val);

        if (mismatches >= DBG_SIMD_MAX_MISM) begin
          $display("[CMP] Alcanzado límite de mismatches (%0d).", DBG_SIMD_MAX_MISM);
          break;
        end
      end
    end

    $display("[CMP] Total mismatches=%0d (de %0d píxeles).", mismatches, limit_pixels);
    if (mismatches == 0)
      $display("[CMP] ¡Perfecto! SEQ y SIMD4 coinciden.");

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
  // Monitor SECUENCIAL (simplificado)
  // --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    if (DBG_SEQ_MONITOR &&
        !dut.mode_simd_eff &&
        dut.out_we_seq) begin

      $display("[SEQ][RUN=%0d][t=%0t] WRITE addr=%0d pix=0x%02h | ox=%0d oy=%0d",
        run_id,
        $time,
        dut.out_waddr_seq,
        dut.out_wdata_seq,
        dut.u_core_seq.ox_cur,
        dut.u_core_seq.oy_cur
      );
    end
  end

  // --------------------------------------------------------------------------
  // Monitor SIMD4 (simplificado)
  // --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    if (dut.mode_simd_eff && DBG_SIMD_SUMMARY) begin
      
      if (dut.out_we_simd0)
        $display("[SIMD][RUN=%0d][t=%0t] L0 WRITE addr=%0d pix=0x%02h ox=%0d oy=%0d",
          run_id, $time, 
          dut.out_waddr_simd0, 
          dut.out_wdata_simd0,
          dut.u_core_simd4.ox_lane[0],
          dut.u_core_simd4.oy_cur
        );

      if (dut.out_we_simd1)
        $display("[SIMD][RUN=%0d][t=%0t] L1 WRITE addr=%0d pix=0x%02h ox=%0d oy=%0d",
          run_id, $time,
          dut.out_waddr_simd1,
          dut.out_wdata_simd1,
          dut.u_core_simd4.ox_lane[1],
          dut.u_core_simd4.oy_cur
        );

      if (dut.out_we_simd2)
        $display("[SIMD][RUN=%0d][t=%0t] L2 WRITE addr=%0d pix=0x%02h ox=%0d oy=%0d",
          run_id, $time,
          dut.out_waddr_simd2,
          dut.out_wdata_simd2,
          dut.u_core_simd4.ox_lane[2],
          dut.u_core_simd4.oy_cur
        );

      if (dut.out_we_simd3)
        $display("[SIMD][RUN=%0d][t=%0t] L3 WRITE addr=%0d pix=0x%02h ox=%0d oy=%0d",
          run_id, $time,
          dut.out_waddr_simd3,
          dut.out_wdata_simd3,
          dut.u_core_simd4.ox_lane[3],
          dut.u_core_simd4.oy_cur
        );
    end
  end

endmodule
