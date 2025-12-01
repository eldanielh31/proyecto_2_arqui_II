`timescale 1ps/1ps

// ============================================================================
// top_sdram_test.sv
//   - Top minimalista para probar la SDRAM externa con sdram_wide_if
//   - Usa sdram_mem_tester para escribir/leer un patrón
//   - LEDs:
//       led_start_on   = test_running
//       led_done       = test_pass (SDRAM OK)
//       led_reset_evt  = test_fail (error)
//       led_simd_mode  = 0
// ============================================================================

module top_sdram_test #(
  parameter int AW = 16
)(
  input  logic clk_50,
  input  logic rst_n,

  output logic led_done,
  output logic led_reset_evt,
  output logic led_start_on,
  output logic led_simd_mode,

  // Puertos físicos de SDRAM externa 64 MB
  output logic [12:0] DRAM_ADDR,
  output logic  [1:0] DRAM_BA,
  output logic        DRAM_CAS_N,
  output logic        DRAM_CKE,
  output logic        DRAM_CLK,
  output logic        DRAM_CS_N,
  inout  wire  [15:0] DRAM_DQ,
  output logic  [1:0] DRAM_DQM,
  output logic        DRAM_RAS_N,
  output logic        DRAM_WE_N
);

  // -------------------------------------------------------------------------
  // Conexión sdram_wide_if (interfaz 32 bits) sobre SDRAM física
  // -------------------------------------------------------------------------
  logic              req_valid;
  logic              req_ready;
  logic              req_we;
  logic [AW-1:0]     req_addr;
  logic [31:0]       req_wdata;
  logic              resp_valid;
  logic [31:0]       resp_rdata;

  sdram_wide_if #(
    .AW       (AW),
    .BASE_ADDR(25'd0)
  ) u_mem_if (
    .clk        (clk_50),
    .rst_n      (rst_n),

    .req_valid  (req_valid),
    .req_ready  (req_ready),
    .req_we     (req_we),
    .req_addr   (req_addr),
    .req_wdata  (req_wdata),
    .resp_valid (resp_valid),
    .resp_rdata (resp_rdata),

    .DRAM_ADDR  (DRAM_ADDR),
    .DRAM_BA    (DRAM_BA),
    .DRAM_CAS_N (DRAM_CAS_N),
    .DRAM_CKE   (DRAM_CKE),
    .DRAM_CLK   (DRAM_CLK),
    .DRAM_CS_N  (DRAM_CS_N),
    .DRAM_DQ    (DRAM_DQ),
    .DRAM_DQM   (DRAM_DQM),
    .DRAM_RAS_N (DRAM_RAS_N),
    .DRAM_WE_N  (DRAM_WE_N)
  );

  // -------------------------------------------------------------------------
  // Tester de SDRAM: escribe y lee TEST_DEPTH palabras
  // -------------------------------------------------------------------------
  logic test_running;
  logic test_pass;
  logic test_fail;

  sdram_mem_tester #(
    .AW         (AW),
    .TEST_DEPTH (256)   // se puede aumentar cuando funcione
  ) u_tester (
    .clk         (clk_50),
    .rst_n       (rst_n),
    .req_valid   (req_valid),
    .req_ready   (req_ready),
    .req_we      (req_we),
    .req_addr    (req_addr),
    .req_wdata   (req_wdata),
    .resp_valid  (resp_valid),
    .resp_rdata  (resp_rdata),
    .test_running(test_running),
    .test_pass   (test_pass),
    .test_fail   (test_fail)
  );

  // -------------------------------------------------------------------------
  // Mapeo de flags a LEDs
  // -------------------------------------------------------------------------
  always_comb begin
    led_start_on   = test_running;
    led_done       = test_pass;
    led_reset_evt  = test_fail;
    led_simd_mode  = 1'b0;
  end

endmodule
