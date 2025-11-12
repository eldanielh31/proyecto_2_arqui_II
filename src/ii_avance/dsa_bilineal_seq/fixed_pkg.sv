package fixed_pkg;
  // Tipos Q8.8
  typedef logic signed [15:0] q88_t;   // Q8.8 signed
  typedef logic        [15:0] uq88_t;  // Q8.8 unsigned (0..1)

  // Convierte Q8.8 -> u8 con redondeo y saturación
  function automatic logic [7:0] q88_to_u8 (input q88_t x);
    logic signed [16:0] xr;      // para sumar 0x80 sin overflow
    logic signed [15:0] shr;
    begin
      xr  = {x[15], x} + 17'sd128; // +0x80
      shr = xr[16:1];              // >>8 con el signo de x preservado
      if (shr < 0)        q88_to_u8 = 8'd0;
      else if (shr > 255) q88_to_u8 = 8'd255;
      else                q88_to_u8 = shr[7:0];
    end
  endfunction

  // Q8.8 * Q8.8 -> Q8.8 con redondeo
  function automatic q88_t mul_q88 (input q88_t a, input q88_t b);
    logic signed [31:0] prod;
    logic signed [31:0] prod_r;
    begin
      prod   = a * b;               // Q16.16
      prod_r = prod + 32'sd128;     // redondeo
      mul_q88 = prod_r[23:8];       // back a Q8.8
    end
  endfunction

  // (1 - ax) en Q8.8 (1.0 = 256)
  function automatic uq88_t one_minus (input uq88_t ax);
    begin
      one_minus = 16'd256 - ax;
    end
  endfunction
endpackage
