`timescale 1ns/1ps
import fixed_pkg::*;

module tb_bilinear_seq;

  // Señales
  logic clk;
  logic rst_n;
  logic start_sw;
  logic led_done;

  // Variables para simulación: DECLARADAS ANTES DE CUALQUIER SENTENCIA
  integer f;
  integer i;

  // Init señales básicas
  initial begin
    clk      = 1'b0;
    rst_n    = 1'b0;
    start_sw = 1'b0;
  end

  // Instancia del DUT
  top_dsa_seq dut (
    .clk_50   (clk),
    .rst_n    (rst_n),
    .start_sw (start_sw),
    .led_done (led_done)
  );

  // Clock 50 MHz
  always #10 clk = ~clk;

  // Estímulos
  initial begin
    // Reset
    #100 rst_n = 1'b1;

    // Cargar imagen de prueba (asegúrate que el archivo esté junto al TB o usa ruta absoluta)
    $readmemh("img_in_64x64.hex", dut.mem_in.mem);

    // Start
    #100 start_sw = 1'b1;
    #40  start_sw = 1'b0;

    // Esperar fin
    wait (led_done == 1'b1);

    // Dump de salida
    f = $fopen("img_out.hex", "w");
    for (i = 0; i < (64*64); i = i + 1) begin
      $fwrite(f, "%02x\n", dut.mem_out.mem[i]);
    end
    $fclose(f);

    $display("DONE");
    $finish;
  end

endmodule
