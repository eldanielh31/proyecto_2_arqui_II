// bilinear_seq.sv — versión corregida (evita init en declaración dentro de always_comb)
`timescale 1ns/1ps
import fixed_pkg::*;

module bilinear_seq #(
  parameter ADDR_W = 19
)(
  input  logic        clk,
  input  logic        rst_n,

  // Config
  input  logic [15:0] in_w,
  input  logic [15:0] in_h,
  input  uq88_t       scale_q88,

  // Control
  input  logic        start,
  output logic        busy,
  output logic        done,

  // BRAM in (read)
  output logic [ADDR_W-1:0] in_addr,
  input  logic [7:0]        in_data,

  // BRAM out (write)
  output logic [ADDR_W-1:0] out_addr,
  output logic [7:0]        out_data,
  output logic              out_we
);

  typedef enum logic [3:0] {
    S_IDLE, S_FETCH00, S_FETCH10, S_FETCH01, S_FETCH11,
    S_MAC, S_WRITE, S_NEXT, S_DONE
  } state_t;

  state_t st, st_n;

  // Coordenadas de salida
  logic [15:0] x_dst, y_dst;

  // out_w/out_h correctos con Q8.8 (sin sobre-escala)
  logic [15:0] out_w, out_h;
  logic [31:0] tmpw, tmph;       // productos 16x16 para tamaño de salida

  // Coordenadas fuente Q8.8
  uq88_t       x_src_q88, y_src_q88;
  logic [15:0] ix, iy;
  uq88_t       ax, ay;
  uq88_t       one_ax, one_ay;

  // vecinos
  logic [7:0] I00, I10, I01, I11;

  // acumulador y productos
  q88_t acc;
  q88_t w00, w10, w01, w11;
  q88_t P00, P10, P01, P11;

  // temporales para multiplicación de coordenadas (declaradas FUERA del always_comb)
  logic [31:0] xs_mul, ys_mul;

  // reciprocal fijo (ej.: 1/0.8 ≈ 1.25 => 320 Q8.8)
  uq88_t reciprocal_q88;
  initial reciprocal_q88 = 16'd320;

  // Helpers
  function automatic [ADDR_W-1:0] addr_of(
    input logic [15:0] x, input logic [15:0] y, input logic [15:0] w
  );
    addr_of = y * w + x;
  endfunction

  // ======= out_w/out_h correctos =======
  // Q8.8: out = floor( in * scale / 256 )
  assign tmpw  = in_w * scale_q88;     // 16*16 → 32, con 8 bits fracc
  assign tmph  = in_h * scale_q88;
  assign out_w = tmpw[23:8];           // >>8
  assign out_h = tmph[23:8];           // >>8

  // Estado
  assign busy = (st != S_IDLE) && (st != S_DONE);
  assign done = (st == S_DONE);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) st <= S_IDLE;
    else        st <= st_n;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x_dst <= 16'd0;
      y_dst <= 16'd0;
      out_we <= 1'b0;
      I00 <= '0; I10 <= '0; I01 <= '0; I11 <= '0;
    end else begin
      out_we <= 1'b0;

      case (st)
        S_IDLE: begin
          if (start) begin
            x_dst <= 16'd0;
            y_dst <= 16'd0;
          end
        end

        // Captura de lecturas (dato llega ciclo siguiente)
        S_FETCH10: I00 <= in_data;
        S_FETCH01: I10 <= in_data;
        S_FETCH11: I01 <= in_data;
        S_MAC:     I11 <= in_data;

        S_WRITE: begin
          out_we <= 1'b1;
        end

        S_NEXT: begin
          if (x_dst + 1 < out_w) begin
            x_dst <= x_dst + 16'd1;
          end else begin
            x_dst <= 16'd0;
            if (y_dst + 1 < out_h) y_dst <= y_dst + 16'd1;
          end
        end

        default: ;
      endcase
    end
  end

  // ======= Combinacional =======
  always_comb begin
    st_n     = st;
    in_addr  = '0;
    out_addr = addr_of(x_dst, y_dst, out_w);

    // defaults (para evitar latches)
    x_src_q88 = '0; y_src_q88 = '0;
    ix = '0; iy = '0; ax = '0; ay = '0; one_ax = '0; one_ay = '0;
    w00 = '0; w10 = '0; w01 = '0; w11 = '0;
    P00 = '0; P10 = '0; P01 = '0; P11 = '0;
    acc = '0;
    xs_mul = '0; ys_mul = '0;

    case (st)
      S_IDLE: begin
        if (start) st_n = S_FETCH00;
      end

      // Calcular coord fuente y primer fetch
      S_FETCH00: begin
        // x_src_q88 = floor( x_dst * reciprocal_q88 / 256 )
        // y_src_q88 = floor( y_dst * reciprocal_q88 / 256 )
        xs_mul    = x_dst * reciprocal_q88;   // 16*16 → 32
        ys_mul    = y_dst * reciprocal_q88;
        x_src_q88 = xs_mul[23:8];
        y_src_q88 = ys_mul[23:8];

        ix = x_src_q88[15:8];
        iy = y_src_q88[15:8];
        ax = {8'd0, x_src_q88[7:0]};
        ay = {8'd0, y_src_q88[7:0]};

        one_ax = one_minus(ax);
        one_ay = one_minus(ay);

        in_addr = addr_of(ix, iy, in_w);
        st_n = S_FETCH10;
      end

      S_FETCH10: begin
        in_addr = addr_of(ix+16'd1, iy, in_w);
        st_n = S_FETCH01;
      end

      S_FETCH01: begin
        in_addr = addr_of(ix, iy+16'd1, in_w);
        st_n = S_FETCH11;
      end

      S_FETCH11: begin
        in_addr = addr_of(ix+16'd1, iy+16'd1, in_w);
        st_n = S_MAC;
      end

      S_MAC: begin
        // pesos
        w00 = mul_q88({1'b0,one_ax}, {1'b0,one_ay});
        w10 = mul_q88({1'b0,ax},     {1'b0,one_ay});
        w01 = mul_q88({1'b0,one_ax}, {1'b0,ay});
        w11 = mul_q88({1'b0,ax},     {1'b0,ay});

        // píxeles promovidos a Q8.8
        P00 = {I00,8'd0};
        P10 = {I10,8'd0};
        P01 = {I01,8'd0};
        P11 = {I11,8'd0};

        // acumulación
        acc = 16'sd0;
        acc = acc + mul_q88(w00, P00);
        acc = acc + mul_q88(w10, P10);
        acc = acc + mul_q88(w01, P01);
        acc = acc + mul_q88(w11, P11);

        st_n = S_WRITE;
      end

      S_WRITE: begin
        out_addr = addr_of(x_dst, y_dst, out_w);
        st_n = S_NEXT;
      end

      S_NEXT: begin
        if ((x_dst + 16'd1 == out_w) && (y_dst + 16'd1 == out_h))
          st_n = S_DONE;
        else
          st_n = S_FETCH00;
      end

      S_DONE: begin
        // permanecer en DONE
      end
    endcase
  end

  // salida u8
  assign out_data = q88_to_u8(acc);

endmodule
