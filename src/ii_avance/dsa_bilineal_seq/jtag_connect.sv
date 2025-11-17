`timescale 1ns/1ps

// Protocolo mínimo (IR=2 bits):
//  IR=2'b01 -> WRITE_REG  (DR = {data[31:0], addr[7:0]}  => [39:8]=data, [7:0]=addr)
//  IR=2'b10 -> READ_REG   (fase 1: DR envía {0, addr}, UPDATE-DR lachea addr;
//                          fase 2: CAPTURE-DR carga {data, addr} y SHIFT-DR la devuelve)
//  IR=2'b00/11 -> NOP

module jtag_connect #(
  parameter int DRW = 40,   // 8b addr + 32b data
  parameter int AW  = 12    // ancho de BRAM (para direccionamiento)
)(
  // Puertos al IP vJTAG
  input  logic                tck,
  input  logic                tdi,
  output logic                tdo,
  input  logic [1:0]          ir_in,    // IR "visible al usuario"
  input  logic [1:0]          ir_out,   // no usado

  // Estados JTAG virtuales
  input  logic                vs_cdr,   // CAPTURE-DR
  input  logic                vs_sdr,   // SHIFT-DR
  input  logic                vs_udr,   // UPDATE-DR

  // Interfaz de registros hacia el core/top
  output logic        start_pulse,       // pulso en clk_sys
  output logic [15:0] cfg_in_w,
  output logic [15:0] cfg_in_h,
  output logic [15:0] cfg_scale_q88,
  input  logic        status_done,

  // === Lectura BRAM de ENTRADA (solo lectura para UI) ===
  output logic [AW-1:0] in_mem_raddr,
  input  logic  [7:0]   in_mem_rdata,

  // === Lectura BRAM de SALIDA (solo lectura para UI) ===
  output logic [AW-1:0] out_mem_raddr,
  input  logic  [7:0]   out_mem_rdata,

  // Reloj del sistema
  input  logic        clk_sys,
  input  logic        rst_sys_n
);

  // Bancos de registro (lado sistema)
  logic [31:0] reg_in_w, reg_in_h, reg_scale, reg_control, reg_status;

  // Direcciones de registros
  localparam byte ADDR_CONTROL     = 8'h00; // bit0: start
  localparam byte ADDR_IN_W        = 8'h01;
  localparam byte ADDR_IN_H        = 8'h02;
  localparam byte ADDR_SCALE_Q88   = 8'h03;
  localparam byte ADDR_STATUS      = 8'h10; // bit0: done

  // Registros para lectura de BRAM de ENTRADA/SALIDA
  localparam byte ADDR_IN_ADDR     = 8'h20; // escribir dirección (LSBs) de mem_in
  localparam byte ADDR_IN_DATA     = 8'h21; // leer dato (8 bits válidos en [7:0])
  localparam byte ADDR_OUT_ADDR    = 8'h30; // escribir dirección (LSBs) de mem_out
  localparam byte ADDR_OUT_DATA    = 8'h31; // leer dato (8 bits válidos en [7:0])

  // ===== Dominio JTAG (tck): shift register DR =====
  logic [DRW-1:0] dr_shift;         // MSB-first por TDI, LSB por TDO
  logic [7:0]     latched_addr;     // addr para lecturas
  logic [31:0]    dr_read_data;

  // Decodificación con ir_in
  wire is_write = (ir_in == 2'b01);
  wire is_read  = (ir_in == 2'b10);

  // CAPTURE-DR: preparar datos de lectura
  always_ff @(posedge tck) begin
    if (vs_cdr) begin
      if (is_read) begin
        // [39:8] = data, [7:0] = addr
        dr_shift <= {dr_read_data, latched_addr};
      end else begin
        dr_shift <= '0;
      end
    end else if (vs_sdr) begin
      // SHIFT-DR: entra MSB por TDI; LSB sale por TDO
      dr_shift <= {tdi, dr_shift[DRW-1:1]};
    end
  end

  // TDO = LSB durante SHIFT-DR
  always_comb begin
    tdo = vs_sdr ? dr_shift[0] : 1'b0;
  end

  // UPDATE-DR (dominio tck): decodificar
  logic        wr_pulse_tck;
  logic [7:0]  wr_addr_hold_tck;
  logic [31:0] wr_data_hold_tck;
  logic        rd_pulse_tck;

  always_ff @(posedge tck) begin
    wr_pulse_tck <= 1'b0;
    rd_pulse_tck <= 1'b0;
    if (vs_udr) begin
      if (is_write) begin
        wr_pulse_tck     <= 1'b1;
        wr_addr_hold_tck <= dr_shift[7:0];
        wr_data_hold_tck <= dr_shift[39:8];
      end else if (is_read) begin
        rd_pulse_tck <= 1'b1;
        latched_addr <= dr_shift[7:0];
      end
    end
  end

  // ===== CDC tck -> clk_sys (handshake por toggle) =====
  logic wr_toggle_tck;
  always_ff @(posedge tck or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      wr_toggle_tck <= 1'b0;
    end else if (wr_pulse_tck) begin
      wr_toggle_tck <= ~wr_toggle_tck;
    end
  end

  // Sincronizadores para toggle
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

  // Sincronizar buses multi-bit (tck -> clk_sys)
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

  // ===== Lado sistema: registros y cableo a BRAMs =====
  logic [31:0] reg_in_addr;       // dirección mem_in (LSBs usados)
  logic [31:0] reg_out_addr;      // dirección mem_out (LSBs usados)
  logic [7:0]  reg_in_data_sys;   // dato capturado desde mem_in (clk_sys)
  logic [7:0]  reg_out_data_sys;  // dato capturado desde mem_out (clk_sys)

  // Direcciones a BRAMs
  assign in_mem_raddr  = reg_in_addr[AW-1:0];
  assign out_mem_raddr = reg_out_addr[AW-1:0];

  // Banco de registros en clk_sys
  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      reg_in_w        <= 32'd64;
      reg_in_h        <= 32'd64;
      reg_scale       <= 32'd205;     // 0.80 Q8.8
      reg_control     <= 32'd0;
      reg_status      <= 32'd0;
      reg_in_addr     <= 32'd0;
      reg_out_addr    <= 32'd0;
      reg_in_data_sys <= 8'd0;
      reg_out_data_sys<= 8'd0;
    end else begin
      if (wr_sys) begin
        unique case (wr_addr_sync2)
          ADDR_IN_W:        reg_in_w      <= wr_data_sync2;
          ADDR_IN_H:        reg_in_h      <= wr_data_sync2;
          ADDR_SCALE_Q88:   reg_scale     <= wr_data_sync2;
          ADDR_CONTROL:     reg_control   <= wr_data_sync2; // bit0=start
          ADDR_IN_ADDR:     reg_in_addr   <= wr_data_sync2;
          ADDR_OUT_ADDR:    reg_out_addr  <= wr_data_sync2;
          default: ;
        endcase
      end
      // status: bit0 = done
      reg_status[0]     <= status_done;
      reg_status[31:1]  <= '0;

      // Captura continua de datos desde BRAMs (1 ciclo tras cambiar addr)
      reg_in_data_sys   <= in_mem_rdata;
      reg_out_data_sys  <= out_mem_rdata;
    end
  end

  // ===== sys -> tck: sincronizar los datos de BRAM para lecturas READ_REG =====
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

  // ===== START pulse ensanchado (8 ciclos) =====
  logic [3:0] start_cnt;
  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      start_cnt <= 4'd0;
    end else begin
      if (wr_sys && (wr_addr_sync2 == ADDR_CONTROL) && wr_data_sync2[0]) begin
        start_cnt <= 4'd8;
      end else if (start_cnt != 4'd0) begin
        start_cnt <= start_cnt - 4'd1;
      end
    end
  end
  assign start_pulse = (start_cnt != 4'd0);

  // Exportar configuraciones al core (truncadas a 16b)
  assign cfg_in_w      = reg_in_w[15:0];
  assign cfg_in_h      = reg_in_h[15:0];
  assign cfg_scale_q88 = reg_scale[15:0];

  // ===== Multiplexor de lectura (lado tck): dr_read_data =====
  always_comb begin
    unique case (latched_addr)
      ADDR_IN_W:        dr_read_data = reg_in_w;
      ADDR_IN_H:        dr_read_data = reg_in_h;
      ADDR_SCALE_Q88:   dr_read_data = reg_scale;
      ADDR_STATUS:      dr_read_data = reg_status;

      // Lectura de BRAM de ENTRADA (8 bits válidos en [7:0] de [39:8])
      ADDR_IN_DATA:     dr_read_data = {24'h0, in_data_sync};

      // Lectura de BRAM de SALIDA (8 bits válidos en [7:0] de [39:8])
      ADDR_OUT_DATA:    dr_read_data = {24'h0, out_data_sync};

      default:          dr_read_data = 32'hDEAD_BEEF;
    endcase
  end

endmodule
