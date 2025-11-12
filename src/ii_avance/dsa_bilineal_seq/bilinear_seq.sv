// ============================================================================
// bilinear_seq.sv  (Quartus 18.0/18.1 friendly, SV mínimo)
// Núcleo secuencial con interpolación bilineal Q8.8 real.
// - Lee las 4 muestras vecinas por píxel (I00, I10, I01, I11) en 4 ciclos.
// - Clamp de bordes para xi, yi y fracciones fx/fy cuando xi/yi están en la
//   última columna/fila.
// - Stride de lectura usa i_in_w (ancho de ENTRADA).
// - out_we escribe el píxel del ciclo actual (fase WRITE).
// - Sin slices sobre concatenaciones temporales ni sintaxis SV avanzada.
// ============================================================================

`timescale 1ns/1ps

module bilinear_seq #(
  parameter AW = 12
)(
  input  wire         clk,
  input  wire         rst_n,

  // Control
  input  wire         start,
  output reg          busy,
  output reg          done,

  // Dimensiones entrada y escala Q8.8
  input  wire [15:0]  i_in_w,
  input  wire [15:0]  i_in_h,
  input  wire [15:0]  i_scale_q88,   // Q8.8, >0

  // Dimensiones salida reportadas
  output reg  [15:0]  o_out_w,
  output reg  [15:0]  o_out_h,

  // Lectura fuente (single-port BRAM like)
  output reg  [AW-1:0] in_raddr,
  input  wire [7:0]    in_rdata,

  // Escritura destino
  output reg  [AW-1:0] out_waddr,
  output reg  [7:0]    out_wdata,
  output reg           out_we
);

  // ---------------- Constantes ----------------
  // ONE_Q08 = 256 en Q0.8
  localparam [8:0] ONE_Q08 = 9'd256;

  // ---------------- Registros de estado ----------------
  reg [15:0] out_w_reg, out_h_reg;
  reg [15:0] ox_cur, oy_cur;

  // Coordenadas fuente acumuladas Q8.8 en 24 bits: [23:8] entero, [7:0] frac
  reg [23:0] sx_fix, sy_fix;

  // inv_scale en Q8.8 (aprox 1/scale)
  reg [15:0] inv_scale_q88;

  // Fracciones Q0.8 (bit-slices directos de registros, NO de expresiones)
  wire [7:0] ax_q = sx_fix[7:0];
  wire [7:0] ay_q = sy_fix[7:0];

  // Enteros (floor)
  wire [15:0] sx_int = sx_fix[23:8];
  wire [15:0] sy_int = sy_fix[23:8];

  // Latch de coordenadas base y fracciones AJUSTADAS (clamp de bordes)
  reg [15:0] xi_base, yi_base;
  reg [7:0]  fx_q, fy_q;

  // Muestras vecinas (latcheadas tras cada lectura)
  reg [7:0] I00, I10, I01, I11;

  // Pesos Q0.16 (wx*wy) a partir de fx_q/fy_q
  wire [8:0]  wx0_ext = ONE_Q08 - {1'b0, fx_q};
  wire [8:0]  wx1_ext = {1'b0, fx_q};
  wire [8:0]  wy0_ext = ONE_Q08 - {1'b0, fy_q};
  wire [8:0]  wy1_ext = {1'b0, fy_q};

  wire [17:0] w00_q016 = wx0_ext * wy0_ext; // Q0.8 * Q0.8 -> Q0.16 (usamos 18b para seguridad)
  wire [17:0] w10_q016 = wx1_ext * wy0_ext;
  wire [17:0] w01_q016 = wx0_ext * wy1_ext;
  wire [17:0] w11_q016 = wx1_ext * wy1_ext;

  // Productos (Q0.16 * u8 -> máx 24b; acumulamos en 32b por simplicidad)
  wire [31:0] p00 = w00_q016 * I00;
  wire [31:0] p10 = w10_q016 * I10;
  wire [31:0] p01 = w01_q016 * I01;
  wire [31:0] p11 = w11_q016 * I11;

  wire [31:0] sum_q016    = p00 + p10 + p01 + p11;
  wire [31:0] sum_rounded = sum_q016 + 32'h0000_8000; // +0.5 (Q0.16)
  wire [7:0]  PIX_next    = sum_rounded[23:16];       // >>16

  // ---------------- FSM ----------------
  localparam S_IDLE       = 4'd0;
  localparam S_INIT       = 4'd1;
  localparam S_ROW_INIT   = 4'd2;
  localparam S_PIXEL_START= 4'd3;
  localparam S_READ00     = 4'd4;
  localparam S_READ10     = 4'd5;
  localparam S_READ01     = 4'd6;
  localparam S_READ11     = 4'd7;
  localparam S_WRITE      = 4'd8;
  localparam S_ADVANCE    = 4'd9;
  localparam S_DONE       = 4'd10;

  reg [3:0] state, state_n;

  // ---------------- Funciones ----------------
  // Dirección lineal (y*width + x) -> AW LSBs
  function [AW-1:0] linaddr;
    input [15:0] x;
    input [15:0] y;
    input [15:0] width;
    reg   [31:0] tmp;
  begin
    tmp     = (y * width) + x;
    linaddr = tmp[AW-1:0];
  end
  endfunction

  // Aproximación a 1/scale en Q8.8: floor((65536 + (scale>>8))/scale)
  function [15:0] inv_q88;
    input [15:0] scale_q88;
    reg   [31:0] num;
  begin
    if (scale_q88 == 16'd0) begin
      inv_q88 = 16'hFFFF;
    end else begin
      num     = 32'd65536 + (scale_q88 >> 8);
      inv_q88 = num / scale_q88;
    end
  end
  endfunction

  // ---------------- Combinacional simple ----------------
  wire [31:0] mul_w = i_in_w * i_scale_q88; // Q8.8
  wire [31:0] mul_h = i_in_h * i_scale_q88; // Q8.8

  // ---------------- Secuencial ----------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state         <= S_IDLE;
      busy          <= 1'b0;
      done          <= 1'b0;
      out_we        <= 1'b0;

      out_waddr     <= {AW{1'b0}};
      out_wdata     <= 8'h00;
      in_raddr      <= {AW{1'b0}};

      ox_cur        <= 16'd0;
      oy_cur        <= 16'd0;

      out_w_reg     <= 16'd0;
      out_h_reg     <= 16'd0;
      inv_scale_q88 <= 16'd0;

      sx_fix        <= 24'd0;
      sy_fix        <= 24'd0;

      o_out_w       <= 16'd0;
      o_out_h       <= 16'd0;

      xi_base       <= 16'd0;
      yi_base       <= 16'd0;
      fx_q          <= 8'd0;
      fy_q          <= 8'd0;

      I00 <= 8'd0; I10 <= 8'd0; I01 <= 8'd0; I11 <= 8'd0;
    end else begin
      state  <= state_n;
      out_we <= 1'b0; // por defecto

      case (state)
        // --- Configuración tras start ---
        S_IDLE: begin
          done <= 1'b0;
          busy <= 1'b0;
          if (start) begin
            out_w_reg     <= mul_w[23:8]; // floor((in*scale)>>8)
            out_h_reg     <= mul_h[23:8];
            o_out_w       <= mul_w[23:8];
            o_out_h       <= mul_h[23:8];
            inv_scale_q88 <= inv_q88(i_scale_q88);
          end
        end

        // --- Inicialización general ---
        S_INIT: begin
          busy   <= 1'b1;
          ox_cur <= 16'd0;
          oy_cur <= 16'd0;
          sx_fix <= 24'd0;     // x = 0.0
          sy_fix <= 24'd0;     // y = 0.0
        end

        // --- Nueva fila ---
        S_ROW_INIT: begin
          ox_cur <= 16'd0;
          sx_fix <= 24'd0;     // reiniciar coordenada X (Q8.8)
        end

        // --- Pixel: preparar xi/yi y fracciones con CLAMP de borde ---
        S_PIXEL_START: begin
          // Base sin clamp
          xi_base <= sx_int;
          yi_base <= sy_int;
          fx_q    <= ax_q;
          fy_q    <= ay_q;

          // Clamp de bordes para bilinear (última col/fila)
          if (sx_int >= i_in_w - 16'd1) begin
            xi_base <= i_in_w - 16'd2;  // garantiza xi+1 válido
            fx_q    <= 8'hFF;           // fracción máxima
          end
          if (sy_int >= i_in_h - 16'd1) begin
            yi_base <= i_in_h - 16'd2;
            fy_q    <= 8'hFF;
          end

          // Programar primera lectura (I00)
          in_raddr <= linaddr(xi_base, yi_base, i_in_w);
        end

        // --- Lecturas secuenciales de las 4 muestras vecinas ---
        S_READ00: begin
          I00     <= in_rdata;
          in_raddr<= linaddr(xi_base + 16'd1, yi_base, i_in_w); // I10
        end

        S_READ10: begin
          I10     <= in_rdata;
          in_raddr<= linaddr(xi_base, yi_base + 16'd1, i_in_w); // I01
        end

        S_READ01: begin
          I01     <= in_rdata;
          in_raddr<= linaddr(xi_base + 16'd1, yi_base + 16'd1, i_in_w); // I11
        end

        S_READ11: begin
          I11     <= in_rdata;
          // nada más aquí; productos se evalúan combinacionalmente
        end

        // --- Escritura del píxel interpolado ---
        S_WRITE: begin
          out_waddr <= linaddr(ox_cur, oy_cur, out_w_reg);
          out_wdata <= PIX_next;
          out_we    <= 1'b1;
        end

        // --- Avance de coordenadas salida y fuente ---
        S_ADVANCE: begin
          // Avance en X si no cerramos fila; si cerramos, avanzar Y
          if (ox_cur + 16'd1 < out_w_reg) begin
            ox_cur <= ox_cur + 16'd1;
            // sx_fix += inv_scale (Q8.8) -> expandir con ceros arriba
            sx_fix <= sx_fix + {8'd0, inv_scale_q88};
          end else begin
            if (oy_cur + 16'd1 < out_h_reg) begin
              oy_cur <= oy_cur + 16'd1;
              // nueva fila: x vuelve a 0; y += inv_scale
              sx_fix <= 24'd0;
              sy_fix <= sy_fix + {8'd0, inv_scale_q88};
            end
          end
        end

        S_DONE: begin
          busy <= 1'b0;
          done <= 1'b1;
        end

        default: begin
          // no-op
        end
      endcase
    end
  end

  // ---------------- Próximo estado ----------------
  always @(*) begin
    state_n = state;
    case (state)
      S_IDLE:         state_n = (start) ? S_INIT : S_IDLE;
      S_INIT:         state_n = S_ROW_INIT;
      S_ROW_INIT:     state_n = S_PIXEL_START;
      S_PIXEL_START:  state_n = S_READ00;
      S_READ00:       state_n = S_READ10;
      S_READ10:       state_n = S_READ01;
      S_READ01:       state_n = S_READ11;
      S_READ11:       state_n = S_WRITE;
      S_WRITE: begin
        // decidir si terminamos, cambiamos de fila o seguimos en la misma fila
        if ((ox_cur + 16'd1 >= out_w_reg) && (oy_cur + 16'd1 >= out_h_reg))
          state_n = S_DONE;
        else
          state_n = S_ADVANCE;
      end
      S_ADVANCE: begin
        if ((ox_cur + 16'd1 >= out_w_reg) && (oy_cur + 16'd1 >= out_h_reg))
          state_n = S_DONE;
        else if (ox_cur + 16'd1 >= out_w_reg)
          state_n = S_ROW_INIT;     // nueva fila
        else
          state_n = S_PIXEL_START;  // siguiente píxel en la misma fila
      end
      S_DONE:         state_n = S_IDLE;
      default:        state_n = S_IDLE;
    endcase
  end

endmodule
