// ============================================================================
// top_dsa_seq.sv  — top con start por switch y LEDs de estado
// - LED done: se queda encendido hasta un nuevo start o reset
// - LED reset_evt: se enciende unos ms después de tocar reset (visual)
// - LED start_on: refleja el estado del switch (ya sincronizado/debounced)
// ============================================================================

`timescale 1ps/1ps
`define MEM_INIT_FILE "img_in_64x64.hex"

module top_dsa_seq
#(
  parameter int AW                = 12,
  // overrideables desde TB
  parameter int DEB_W             = 20,      // debounce (~10–20 ms @50MHz)
  parameter int RST_STRETCH_W     = 22,      // pulso visual post-reset
  // parámetros de tamaño/escala (overrideables si se desea)
  parameter int unsigned IN_W_INIT      = 16'd64,
  parameter int unsigned IN_H_INIT      = 16'd64,
  parameter int unsigned SCALE_Q88_INIT = 16'd205
)(
  input  logic clk_50,
  input  logic rst_n,          // reset asíncrono activo bajo (de la placa)
  input  logic start_sw,       // switch físico de inicio (nivel)

  output logic led_done,       // latched: ON cuando termina, hasta próximo start/reset
  output logic led_reset_evt,  // ON por ventana corta tras tocar reset
  output logic led_start_on    // refleja el start_sw filtrado/sincronizado
);

  // ---------------- Señales core ----------------
  logic        start_pulse;
  logic        busy, done;

  logic [15:0] in_w, in_h, scale_q88;
  logic [15:0] out_w_s, out_h_s;

  logic [AW-1:0] in_raddr;
  logic [7:0]    in_rdata;

  logic [AW-1:0] out_waddr;
  logic [7:0]    out_wdata;
  logic          out_we;

  // Parámetros fijos (pueden cambiarse por JTAG en otra variante)
  assign in_w      = IN_W_INIT;
  assign in_h      = IN_H_INIT;
  assign scale_q88 = SCALE_Q88_INIT;

  // =========================================================================
  // 1) Sincronizador + antirrebote simple del switch de start
  // =========================================================================
  // Dos flip-flops de sincronización
  logic sw_meta, sw_sync;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      sw_meta <= 1'b0;
      sw_sync <= 1'b0;
    end else begin
      sw_meta <= start_sw;
      sw_sync <= sw_meta;
    end
  end

  // Antirrebote básico por contador: requiere nivel estable N ciclos
  logic [DEB_W-1:0] deb_cnt;
  logic             sw_debounced, sw_debounced_q;

  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      deb_cnt        <= '0;
      sw_debounced   <= 1'b0;
      sw_debounced_q <= 1'b0;
    end else begin
      // reinicia el contador si el estado del switch cambia
      if (sw_sync != sw_debounced) begin
        deb_cnt <= '0;
      end else if (deb_cnt != {DEB_W{1'b1}}) begin
        deb_cnt <= deb_cnt + 1'b1;
      end
      // cuando el nivel se mantiene estable suficiente tiempo, actualiza
      if (deb_cnt == {DEB_W{1'b1}}) begin
        sw_debounced <= sw_sync;
      end
      // retardo para flanco
      sw_debounced_q <= sw_debounced;
    end
  end

  // Pulso de start en flanco ascendente del switch debounced
  assign start_pulse = (sw_debounced & ~sw_debounced_q);

  // LED que refleja el nivel del start ya filtrado/sincronizado
  assign led_start_on = sw_debounced;

  // =========================================================================
  // 2) LED de evento de reset (latido corto al soltar reset)
  // =========================================================================
  // Encender por ~X ms después de salir de reset para visualizar que se tocó.
  logic [RST_STRETCH_W-1:0] rst_cnt;
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      // al entrar en reset, precargue el contador al máximo
      rst_cnt <= {RST_STRETCH_W{1'b1}};
    end else begin
      if (rst_cnt != '0) rst_cnt <= rst_cnt - 1'b1;
    end
  end
  assign led_reset_evt = (rst_cnt != '0);

  // =========================================================================
  // 3) Memorias on-chip
  // =========================================================================
  // Entrada: inicializada desde HEX vía macro MEM_INIT_FILE (ver .qsf o defines)
  onchip_mem_img #(
    .ADDR_W (AW),
    .INIT_EN(1'b1)           // cargar imagen desde archivo HEX / MIF
  ) mem_in (
    .clk   (clk_50),
    .raddr (in_raddr),
    .rdata (in_rdata),
    .waddr ('0),
    .wdata ('0),
    .we    (1'b0)
  );

  // Salida: escritura desde el core; lectura no usada aquí
  onchip_mem_img #(
    .ADDR_W (AW),
    .INIT_EN(1'b0)
  ) mem_out (
    .clk   (clk_50),
    .raddr ('0),
    .rdata (),
    .waddr (out_waddr),
    .wdata (out_wdata),
    .we    (out_we)
  );

  // =========================================================================
  // 4) Núcleo bilineal
  // =========================================================================
  bilinear_seq #(.AW(AW)) core (
    .clk         (clk_50),
    .rst_n       (rst_n),
    .start       (start_pulse),
    .busy        (busy),
    .done        (done),

    .i_in_w      (in_w),
    .i_in_h      (in_h),
    .i_scale_q88 (scale_q88),

    .o_out_w     (out_w_s),
    .o_out_h     (out_h_s),

    .in_raddr    (in_raddr),
    .in_rdata    (in_rdata),

    .out_waddr   (out_waddr),
    .out_wdata   (out_wdata),
    .out_we      (out_we)
  );

  // =========================================================================
  // 5) LED done latcheado: queda ON hasta nuevo start o reset
  // =========================================================================
  always_ff @(posedge clk_50 or negedge rst_n) begin
    if (!rst_n) begin
      led_done <= 1'b0;
    end else begin
      // nuevo start apaga el LED para la siguiente corrida
      if (start_pulse)
        led_done <= 1'b0;
      else if (done)
        led_done <= 1'b1;
    end
  end

endmodule
