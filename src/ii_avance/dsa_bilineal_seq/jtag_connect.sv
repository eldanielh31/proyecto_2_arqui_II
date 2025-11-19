`timescale 1ns/1ps
//
// jtag_connect.sv — Puente Virtual JTAG (IR=2 bits)
//   IR=2'b01 -> WRITE_REG  (DR = {data[31:0], addr[7:0]} => [39:8]=data, [7:0]=addr)
//   IR=2'b10 -> READ_REG   (F1: {0,addr}; F2: CAPTURE-DR carga {data,addr} y SHIFT-DR la devuelve)
//
module jtag_connect #(
  parameter int DRW = 40,   // 8b addr + 32b data
  parameter int AW  = 12    // ancho de BRAM (para direccionamiento)
)(
  // vJTAG primitives
  input  logic                tck,
  input  logic                tdi,
  output logic                tdo,
  input  logic [1:0]          ir_in,

  input  logic                vs_cdr,
  input  logic                vs_sdr,
  input  logic                vs_udr,

  // Hacia el core/top
  output logic        start_pulse,       // pulso en clk_sys (8 ciclos)
  output logic [15:0] cfg_in_w,
  output logic [15:0] cfg_in_h,
  output logic [15:0] cfg_scale_q88,
  input  logic        status_done,
  input  logic        status_busy,
  input  logic [31:0] perf_flops,
  input  logic [31:0] perf_mem_rd,
  input  logic [31:0] perf_mem_wr,

  // BRAM ENTRADA: lectura (UI)
  output logic [AW-1:0] in_mem_raddr,
  input  logic  [7:0]   in_mem_rdata,

  // BRAM ENTRADA: escritura (upload desde UI)
  output logic [AW-1:0] in_mem_waddr,
  output logic  [7:0]   in_mem_wdata,
  output logic          in_mem_we,

  // BRAM SALIDA: lectura (UI)
  output logic [AW-1:0] out_mem_raddr,
  input  logic  [7:0]   out_mem_rdata,

  // Reloj sistema
  input  logic          clk_sys,
  input  logic          rst_sys_n
);

  // ========= Direcciones de registros =========
  localparam byte ADDR_CONTROL     = 8'h00; // bit0: start
  localparam byte ADDR_IN_W        = 8'h01;
  localparam byte ADDR_IN_H        = 8'h02;
  localparam byte ADDR_SCALE_Q88   = 8'h03;

  // Estado y performance
  localparam byte ADDR_STATUS      = 8'h10; // bit0: done, bit1: busy, bit2: error
  localparam byte ADDR_PERF_FLOPS  = 8'h11;
  localparam byte ADDR_PERF_MEM_RD = 8'h12;
  localparam byte ADDR_PERF_MEM_WR = 8'h13;
  localparam byte ADDR_PROGRESS    = 8'h14; // progreso ≈ píxeles escritos

  // BRAM IN (view)
  localparam byte ADDR_IN_ADDR     = 8'h20; // set raddr
  localparam byte ADDR_IN_DATA     = 8'h21; // read data (8b válidos en [7:0])

  // BRAM OUT (view)
  localparam byte ADDR_OUT_ADDR    = 8'h30; // set raddr
  localparam byte ADDR_OUT_DATA    = 8'h31; // read data (8b válidos en [7:0])

  // BRAM IN (upload)
  localparam byte ADDR_IN_WADDR    = 8'h22; // set waddr
  localparam byte ADDR_IN_WDATA    = 8'h23; // write byte y auto-incrementa waddr

  // ========= Cuantización de factor de escala Q8.8 =========
  function automatic [15:0] quantize_scale_q88(input [15:0] raw);
    localparam [15:0] SCALE_MIN_Q88  = 16'd128; // 0.50 * 256
    localparam [15:0] SCALE_MAX_Q88  = 16'd256; // 1.00 * 256
    localparam [15:0] SCALE_STEP_Q88 = 16'd13;  // ≈ 0.05 * 256 ≈ 12.8

    reg [15:0] val;
    reg [15:0] delta;
    reg [15:0] k;
  begin
    // Clamping al rango [0.5, 1.0]
    if (raw < SCALE_MIN_Q88)      val = SCALE_MIN_Q88;
    else if (raw > SCALE_MAX_Q88) val = SCALE_MAX_Q88;
    else                          val = raw;

    // Distancia desde 0.5
    delta = val - SCALE_MIN_Q88;

    // k = round( delta / STEP )  → 0..10
    k = (delta + (SCALE_STEP_Q88 >> 1)) / SCALE_STEP_Q88;
    if (k > 16'd10) k = 16'd10;

    quantize_scale_q88 = SCALE_MIN_Q88 + k * SCALE_STEP_Q88;
  end
  endfunction

  // ========= Dominio JTAG (tck) =========
  logic [DRW-1:0] dr_shift;
  logic [7:0]     latched_addr;
  logic [31:0]    dr_read_data;

  wire is_write = (ir_in == 2'b01);
  wire is_read  = (ir_in == 2'b10);

  always_ff @(posedge tck) begin
    if (vs_cdr) begin
      dr_shift <= is_read ? {dr_read_data, latched_addr} : '0;
    end else if (vs_sdr) begin
      dr_shift <= {tdi, dr_shift[DRW-1:1]};
    end
  end

  always_comb begin
    tdo = vs_sdr ? dr_shift[0] : 1'b0;
  end

  // UPDATE-DR en tck
  logic        wr_pulse_tck;
  logic [7:0]  wr_addr_hold_tck;
  logic [31:0] wr_data_hold_tck;

  always_ff @(posedge tck) begin
    wr_pulse_tck <= 1'b0;
    if (vs_udr) begin
      if (is_write) begin
        wr_pulse_tck     <= 1'b1;
        wr_addr_hold_tck <= dr_shift[7:0];
        wr_data_hold_tck <= dr_shift[39:8];
      end else if (is_read) begin
        latched_addr <= dr_shift[7:0];
      end
    end
  end

  // ========= CDC tck -> clk_sys =========
  logic wr_toggle_tck;
  always_ff @(posedge tck or negedge rst_sys_n) begin
    if (!rst_sys_n) wr_toggle_tck <= 1'b0;
    else if (wr_pulse_tck) wr_toggle_tck <= ~wr_toggle_tck;
  end

  logic wr_tog_meta, wr_tog_sync, wr_tog_sync_d;
  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      wr_tog_meta   <= 1'b0;
      wr_tog_sync   <= 1'b0;
      wr_tog_sync_d <= 1'b0;
    end else begin
      wr_tog_meta   <= wr_toggle_tck;
      wr_tog_sync   <= wr_tog_meta;
      wr_tog_sync_d <= wr_tog_sync;
    end
  end
  wire wr_sys = (wr_tog_sync ^ wr_tog_sync_d);

  // buses multi-bit
  logic [7:0]  wr_addr_sync1, wr_addr_sync2;
  logic [31:0] wr_data_sync1, wr_data_sync2;
  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      wr_addr_sync1 <= '0; wr_addr_sync2 <= '0;
      wr_data_sync1 <= '0; wr_data_sync2 <= '0;
    end else begin
      wr_addr_sync1 <= wr_addr_hold_tck;
      wr_addr_sync2 <= wr_addr_sync1;
      wr_data_sync1 <= wr_data_hold_tck;
      wr_data_sync2 <= wr_data_sync1;
    end
  end

  // ========= Lado sistema =========
  logic [31:0] reg_in_w, reg_in_h, reg_scale;
  logic [31:0] reg_status;
  logic [31:0] reg_in_raddr, reg_out_raddr;
  logic [31:0] reg_in_waddr;           // único driver
  logic [7:0]  reg_in_data_sys, reg_out_data_sys;

  logic [31:0] reg_perf_flops, reg_perf_mem_rd, reg_perf_mem_wr;
  logic [31:0] reg_progress;

  assign in_mem_raddr  = reg_in_raddr[AW-1:0];
  assign out_mem_raddr = reg_out_raddr[AW-1:0];

  // START pulse (8 ciclos)
  logic [3:0] start_cnt;
  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) start_cnt <= 4'd0;
    else begin
      if (wr_sys && (wr_addr_sync2 == ADDR_CONTROL) && wr_data_sync2[0]) start_cnt <= 4'd8;
      else if (start_cnt != 4'd0) start_cnt <= start_cnt - 4'd1;
    end
  end
  assign start_pulse = (start_cnt != 4'd0);

  // Banco de registros + subida de imagen
  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      reg_in_w         <= 32'd64;
      reg_in_h         <= 32'd64;
      reg_scale        <= 32'd205;      // 0.80 Q8.8 aprox.
      reg_status       <= 32'd0;

      reg_in_raddr     <= 32'd0;
      reg_out_raddr    <= 32'd0;

      reg_in_waddr     <= 32'd0;
      in_mem_we        <= 1'b0;
      in_mem_waddr     <= '0;
      in_mem_wdata     <= 8'h00;

      reg_in_data_sys  <= 8'd0;
      reg_out_data_sys <= 8'd0;

      reg_perf_flops   <= 32'd0;
      reg_perf_mem_rd  <= 32'd0;
      reg_perf_mem_wr  <= 32'd0;
      reg_progress     <= 32'd0;
    end else begin
      in_mem_we <= 1'b0;  // por defecto

      if (wr_sys) begin
        unique case (wr_addr_sync2)
          ADDR_IN_W:        reg_in_w      <= wr_data_sync2;
          ADDR_IN_H:        reg_in_h      <= wr_data_sync2;
          ADDR_SCALE_Q88:   reg_scale     <= {16'd0, quantize_scale_q88(wr_data_sync2[15:0])};
          ADDR_CONTROL:     /* sin latch; solo start_cnt */ ;
          ADDR_IN_ADDR:     reg_in_raddr  <= wr_data_sync2;
          ADDR_OUT_ADDR:    reg_out_raddr <= wr_data_sync2;

          // upload a mem_in:
          ADDR_IN_WADDR:    reg_in_waddr  <= wr_data_sync2;
          ADDR_IN_WDATA: begin
            in_mem_we    <= 1'b1;
            in_mem_waddr <= reg_in_waddr[AW-1:0];
            in_mem_wdata <= wr_data_sync2[7:0];
            reg_in_waddr <= reg_in_waddr + 32'd1; // autoincremento
          end
          default: ;
        endcase
      end

      // status: bit0=done, bit1=busy, bit2=error (sin uso por ahora)
      reg_status[0]     <= status_done;
      reg_status[1]     <= status_busy;
      reg_status[2]     <= 1'b0;
      reg_status[31:3]  <= '0;

      // performance y progreso (reflejados desde el core/top)
      reg_perf_flops    <= perf_flops;
      reg_perf_mem_rd   <= perf_mem_rd;
      reg_perf_mem_wr   <= perf_mem_wr;
      reg_progress      <= perf_mem_wr; // progreso ≈ píxeles escritos

      // captura continua desde BRAMs
      reg_in_data_sys   <= in_mem_rdata;
      reg_out_data_sys  <= out_mem_rdata;
    end
  end

  // export a core
  assign cfg_in_w      = reg_in_w[15:0];
  assign cfg_in_h      = reg_in_h[15:0];
  assign cfg_scale_q88 = reg_scale[15:0];

  // tck: sincronizar datos de BRAMs para lecturas
  logic [7:0] in_data_meta,  in_data_sync;
  logic [7:0] out_data_meta, out_data_sync;
  always_ff @(posedge tck or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      in_data_meta  <= '0; in_data_sync  <= '0;
      out_data_meta <= '0; out_data_sync <= '0;
    end else begin
      in_data_meta  <= reg_in_data_sys;
      in_data_sync  <= in_data_meta;
      out_data_meta <= reg_out_data_sys;
      out_data_sync <= out_data_meta;
    end
  end

  // Multiplexor de lectura
  always_comb begin
    unique case (latched_addr)
      ADDR_IN_W:        dr_read_data = reg_in_w;
      ADDR_IN_H:        dr_read_data = reg_in_h;
      ADDR_SCALE_Q88:   dr_read_data = reg_scale;
      ADDR_STATUS:      dr_read_data = reg_status;

      ADDR_PERF_FLOPS:  dr_read_data = reg_perf_flops;
      ADDR_PERF_MEM_RD: dr_read_data = reg_perf_mem_rd;
      ADDR_PERF_MEM_WR: dr_read_data = reg_perf_mem_wr;
      ADDR_PROGRESS:    dr_read_data = reg_progress;

      ADDR_IN_DATA:     dr_read_data = {24'h0, in_data_sync};
      ADDR_OUT_DATA:    dr_read_data = {24'h0, out_data_sync};

      default:          dr_read_data = 32'hDEAD_BEEF;
    endcase
  end

endmodule
