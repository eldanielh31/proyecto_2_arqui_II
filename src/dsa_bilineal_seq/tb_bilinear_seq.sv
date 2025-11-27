`timescale 1ps/1ps

// ============================================================================
// tb_bilinear_seq — Testbench para top_dsa_wide
//   - Ejecuta primero el núcleo SECUENCIAL (mode_simd_sw = 0).
//   - Luego hace reset y ejecuta el núcleo SIMD4 (mode_simd_sw = 1).
//   - Genera clock, reset y pulso de start.
//   - Carga la imagen de entrada desde archivo HEX en formato wide (32-bit).
//   - Espera a 'done' y vuelca mem_out a archivos HEX (seq y simd).
//   - Compara mem_out de SEQ vs mem_out de SIMD4 (golden vs DUT SIMD).
//   - Monitores de debug para SEQ y SIMD.
// ============================================================================

module tb_bilinear_seq;

  // --------------------------------------------------------------------------
  // Parámetros
  // --------------------------------------------------------------------------
  localparam int    AW             = 10;          // Reducido por wide memory (4 pixels/word)
  localparam int    MEM_DEPTH      = (1 << AW);  // Depth in words (not bytes)
  localparam time   T_CLK_PS       = 20_000;      // 20 ns -> 50 MHz

  // Configuración de imagen / escala
  localparam int    IN_W_CFG       = 16;          // ancho de la imagen de entrada
  localparam int    IN_H_CFG       = 16;          // alto de la imagen de entrada
  localparam int    SCALE_Q88_CFG  = 16'd205;     // factor Q8.8 (~0.8)

  localparam string IN_HEX_FILE    = "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_16x16.hex";

  // Flags de debug
  localparam bit DBG_SEQ_MONITOR      = 1'b1;
  localparam bit DBG_SIMD_SUMMARY     = 1'b1;
  localparam bit DBG_SIMD_DETAILED    = 1'b1;
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
  // Buffer golden para salida SECUENCIAL (en formato byte)
  // --------------------------------------------------------------------------
  logic [7:0] golden_seq [0:(MEM_DEPTH*4)-1];  // *4 porque cada word tiene 4 bytes

  // Helper arrays for converting wide memory to byte array
  logic [7:0] byte_image_in [0:(IN_W_CFG*IN_H_CFG)-1];

  // --------------------------------------------------------------------------
  // DUT
  // --------------------------------------------------------------------------
  top_dsa_seq #(
    .AW            (AW),
    .SIMULATION    (1),
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
  // Función auxiliar para dirección lineal
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
  // Function: Extract byte from wide word
  // --------------------------------------------------------------------------
  function automatic logic [7:0] extract_byte_from_word(
    input logic [31:0] word,
    input logic [1:0]  byte_offset
  );
  begin
    case (byte_offset)
      2'b00: extract_byte_from_word = word[7:0];
      2'b01: extract_byte_from_word = word[15:8];
      2'b10: extract_byte_from_word = word[23:16];
      2'b11: extract_byte_from_word = word[31:24];
    endcase
  end
  endfunction

  // --------------------------------------------------------------------------
  // Function: Get pixel from wide memory
  // --------------------------------------------------------------------------
  function automatic logic [7:0] get_pixel_from_wide_mem(
    input int pixel_addr,
    input logic [31:0] mem [0:MEM_DEPTH-1]
  );
    int word_addr;
    logic [1:0] byte_offset;
  begin
    word_addr = pixel_addr >> 2;  // Divide by 4
    byte_offset = pixel_addr[1:0];
    if (word_addr < MEM_DEPTH)
      get_pixel_from_wide_mem = extract_byte_from_word(mem[word_addr], byte_offset);
    else
      get_pixel_from_wide_mem = 8'h00;
  end
  endfunction

  // --------------------------------------------------------------------------
  // Task: volcar mem_out a archivo HEX (byte format)
  // --------------------------------------------------------------------------
  task automatic dump_mem_out(
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
      disable dump_mem_out;
    end

    $display("[TB] Volcando mem_out (%0dx%0d) a '%s'", ow_local, oh_local, fname);

    for (y = 0; y < oh_local; y = y + 1) begin
      for (x = 0; x < ow_local; x = x + 1) begin
        pixel_addr = y * ow_local + x;
        val = get_pixel_from_wide_mem(pixel_addr, dut.mem_out.mem);
        $fdisplay(fd, "%02h", val);
      end
    end

    $fclose(fd);
    $display("[TB] Dump completado.");
  end
  endtask

  // --------------------------------------------------------------------------
  // Task: Load byte-format HEX into wide memory
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
      
      // Convert byte array to wide memory format
      for (i = 0; i < (IN_W_CFG * IN_H_CFG); i = i + 4) begin
        word_idx = i >> 2;
        
        temp_word[7:0]   = (i+0 < IN_W_CFG*IN_H_CFG) ? byte_image_in[i+0] : 8'h00;
        temp_word[15:8]  = (i+1 < IN_W_CFG*IN_H_CFG) ? byte_image_in[i+1] : 8'h00;
        temp_word[23:16] = (i+2 < IN_W_CFG*IN_H_CFG) ? byte_image_in[i+2] : 8'h00;
        temp_word[31:24] = (i+3 < IN_W_CFG*IN_H_CFG) ? byte_image_in[i+3] : 8'h00;
        
        dut.mem_in.mem[word_idx] = temp_word;
      end
      
      $display("[TB] Imagen cargada. Primeros 4 words (16 bytes):");
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

    // Inicializar golden_seq
    for (i = 0; i < (MEM_DEPTH*4); i = i + 1)
      golden_seq[i] = 8'h00;

    // Reset inicial
    #100_000;
    rst_n = 1'b1;

    // Esperar a fin de limpieza
    @(negedge dut.clear_active);
    repeat (5) @(posedge clk_50);

    // Cargar imagen
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
             dut.perf_flops,
             dut.perf_mem_rd,
             dut.perf_mem_wr);

    dump_mem_out(
      "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_out_seq_wide.hex",
      ow, oh
    );

    // Guardar golden (byte por byte desde wide memory)
    for (i = 0; i < (ow * oh); i = i + 1) begin
      golden_seq[i] = get_pixel_from_wide_mem(i, dut.mem_out.mem);
    end

    $display("[TB][SEQ] Golden sequence guardada (%0d pixels)", ow*oh);

    // ======================================================
    // Reset entre corridas
    // ======================================================
    $display("\n[TB] ===== RESETEANDO PARA CORRIDA 2: SIMD4 ===== t=%0t\n", $time);

    rst_n = 1'b0;
    #100_000;
    rst_n = 1'b1;

    @(negedge dut.clear_active);
    repeat (5) @(posedge clk_50);

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
             dut.perf_flops,
             dut.perf_mem_rd,
             dut.perf_mem_wr);

    dump_mem_out(
      "C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/img_out_simd_wide.hex",
      ow, oh
    );

    // ------------------------------------------------------------------
    // Comparación final SEQ vs SIMD4
    // ------------------------------------------------------------------
    mismatches   = 0;
    limit_pixels = ow * oh;

    $display("\n[TB][CMP] Comparando golden_seq (SEQ) vs mem_out (SIMD4) en %0d píxeles...", 
             limit_pixels);

    for (i = 0; i < limit_pixels; i = i + 1) begin
      automatic logic [7:0] simd_val = get_pixel_from_wide_mem(i, dut.mem_out.mem);
      
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
  // Monitor SECUENCIAL
  //   - Log de vecinos cuando mem_data_valid
  //   - Log de cada write (todas las direcciones)
// --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    // Vecinos leídos de memoria
    if (DBG_SEQ_MONITOR &&
        !dut.mode_simd_eff &&
        dut.u_core_seq.mem_data_valid) begin
      $display("[SEQ][MEM][t=%0t] ox=%0d oy=%0d xi=%0d yi=%0d fx=%0d fy=%0d TL=0x%02h TR=0x%02h BL=0x%02h BR=0x%02h",
        $time,
        dut.u_core_seq.ox_cur,
        dut.u_core_seq.oy_cur,
        dut.u_core_seq.xi_base,
        dut.u_core_seq.yi_base,
        dut.u_core_seq.fx_q,
        dut.u_core_seq.fy_q,
        dut.u_core_seq.mem_pixel_tl,
        dut.u_core_seq.mem_pixel_tr,
        dut.u_core_seq.mem_pixel_bl,
        dut.u_core_seq.mem_pixel_br
      );
    end

    // Escritura de salida (todas las direcciones)
    if (DBG_SEQ_MONITOR &&
        !dut.mode_simd_eff &&
        dut.u_core_seq.out_we) begin

      $display("[SEQ][RUN=%0d][t=%0t] WRITE addr=%0d pix=0x%02h | ox=%0d oy=%0d | sx_int=%0d sy_int=%0d ax=%0d ay=%0d | xi=%0d yi=%0d fx=%0d fy=%0d",
        run_id,
        $time,
        dut.u_core_seq.out_waddr,
        dut.u_core_seq.out_wdata,
        dut.u_core_seq.ox_cur,
        dut.u_core_seq.oy_cur,
        dut.u_core_seq.sx_int,
        dut.u_core_seq.sy_int,
        dut.u_core_seq.ax_q,
        dut.u_core_seq.ay_q,
        dut.u_core_seq.xi_base,
        dut.u_core_seq.yi_base,
        dut.u_core_seq.fx_q,
        dut.u_core_seq.fy_q
      );
    end
    
    // Debug: Monitor FSM state (al inicio para referencia)
    if (DBG_SEQ_MONITOR && !dut.mode_simd_eff && (cycles < 200)) begin
      $display("[SEQ][DBG][t=%0t cyc=%0d] state=%0d busy=%0d done=%0d",
        $time, cycles,
        dut.u_core_seq.state,
        dut.u_core_seq.busy,
        dut.u_core_seq.done
      );
    end
  end

  // --------------------------------------------------------------------------
  // Monitor SIMD4
  // --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    if (dut.mode_simd_eff && DBG_SIMD_SUMMARY) begin
      
      if (dut.u_core_simd4.out_we0)
        $display("[SIMD][RUN=%0d][t=%0t] L0 WRITE addr=%0d pix=0x%02h ox=%0d oy=%0d xi=%0d yi=%0d fx=%0d fy=%0d",
          run_id, $time, 
          dut.u_core_simd4.out_waddr0, 
          dut.u_core_simd4.out_wdata0,
          dut.u_core_simd4.ox_lane[0],
          dut.u_core_simd4.oy_cur,
          dut.u_core_simd4.xi_base_lane[0],
          dut.u_core_simd4.yi_base_row,
          dut.u_core_simd4.fx_q_lane[0],
          dut.u_core_simd4.fy_q_row
        );

      if (dut.u_core_simd4.out_we1)
        $display("[SIMD][RUN=%0d][t=%0t] L1 WRITE addr=%0d pix=0x%02h ox=%0d oy=%0d xi=%0d yi=%0d fx=%0d fy=%0d",
          run_id, $time,
          dut.u_core_simd4.out_waddr1,
          dut.u_core_simd4.out_wdata1,
          dut.u_core_simd4.ox_lane[1],
          dut.u_core_simd4.oy_cur,
          dut.u_core_simd4.xi_base_lane[1],
          dut.u_core_simd4.yi_base_row,
          dut.u_core_simd4.fx_q_lane[1],
          dut.u_core_simd4.fy_q_row
        );

      if (dut.u_core_simd4.out_we2)
        $display("[SIMD][RUN=%0d][t=%0t] L2 WRITE addr=%0d pix=0x%02h ox=%0d oy=%0d xi=%0d yi=%0d fx=%0d fy=%0d",
          run_id, $time,
          dut.u_core_simd4.out_waddr2,
          dut.u_core_simd4.out_wdata2,
          dut.u_core_simd4.ox_lane[2],
          dut.u_core_simd4.oy_cur,
          dut.u_core_simd4.xi_base_lane[2],
          dut.u_core_simd4.yi_base_row,
          dut.u_core_simd4.fx_q_lane[2],
          dut.u_core_simd4.fy_q_row
        );

      if (dut.u_core_simd4.out_we3)
        $display("[SIMD][RUN=%0d][t=%0t] L3 WRITE addr=%0d pix=0x%02h ox=%0d oy=%0d xi=%0d yi=%0d fx=%0d fy=%0d",
          run_id, $time,
          dut.u_core_simd4.out_waddr3,
          dut.u_core_simd4.out_wdata3,
          dut.u_core_simd4.ox_lane[3],
          dut.u_core_simd4.oy_cur,
          dut.u_core_simd4.xi_base_lane[3],
          dut.u_core_simd4.yi_base_row,
          dut.u_core_simd4.fx_q_lane[3],
          dut.u_core_simd4.fy_q_row
        );
    end

    // Detailed monitoring
    if (dut.mode_simd_eff && DBG_SIMD_DETAILED) begin
      // Monitor memory controller activity
      if (dut.u_core_simd4.u_mem_ctrl.state == dut.u_core_simd4.u_mem_ctrl.DATA_READY &&
          dut.u_core_simd4.current_lane < 4) begin
        
        $display("[SIMD][MEM_CTRL][t=%0t] Lane %0d data ready: TL=0x%02h TR=0x%02h BL=0x%02h BR=0x%02h | xi=%0d yi=%0d fx=%0d fy=%0d",
          $time,
          dut.u_core_simd4.current_lane,
          dut.u_core_simd4.u_mem_ctrl.pixel_tl,
          dut.u_core_simd4.u_mem_ctrl.pixel_tr,
          dut.u_core_simd4.u_mem_ctrl.pixel_bl,
          dut.u_core_simd4.u_mem_ctrl.pixel_br,
          dut.u_core_simd4.xi_base_lane[dut.u_core_simd4.current_lane],
          dut.u_core_simd4.yi_base_row,
          dut.u_core_simd4.fx_q_lane[dut.u_core_simd4.current_lane],
          dut.u_core_simd4.fy_q_row
        );
      end
    end
  end

  // --------------------------------------------------------------------------
  // Monitor memory read addresses (detailed debug)
  // --------------------------------------------------------------------------
  always @(posedge clk_50) begin
    if (DBG_SIMD_DETAILED && dut.mode_simd_eff) begin
      if (dut.u_core_simd4.u_mem_ctrl.state == dut.u_core_simd4.u_mem_ctrl.READ_REQ) begin
        $display("[MEM_READ][t=%0t] raddr0=0x%03h raddr1=0x%03h",
          $time,
          dut.mem_in_raddr0,
          dut.mem_in_raddr1
        );
      end
    end
  end

endmodule
