`timescale 1ns/1ps

module jtag_connect #(
  parameter int DRW = 40,   // 8b addr + 32b data
  parameter int AW  = 16    // ancho de dirección de las memorias
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
  output logic        start_pulse,       // pulso en clk_sys
  output logic [15:0] cfg_in_w,
  output logic [15:0] cfg_in_h,
  output logic [15:0] cfg_scale_q88,
  output logic        cfg_mode_simd,

  input  logic        status_done,
  input  logic        status_busy,
  input  logic [31:0] perf_flops,
  input  logic [31:0] perf_mem_rd,
  input  logic [31:0] perf_mem_wr,

  // dimensiones de salida del core
  input  logic [15:0] out_w,
  input  logic [15:0] out_h,

  // Memoria de entrada ancha (32 bits) — interfaz física
  output logic              mem_req_valid,
  output logic              mem_req_we,
  output logic [AW-1:0]     mem_req_addr,
  output logic [31:0]       mem_req_wdata,
  input  logic              mem_req_ready,
  input  logic              mem_resp_valid,
  input  logic [31:0]       mem_resp_rdata,

  // Memoria de salida (8 bits) — sólo lectura
  output logic [AW-1:0]     out_mem_raddr,
  input  logic  [7:0]       out_mem_rdata,

  // Reloj sistema
  input  logic          clk_sys,
  input  logic          rst_sys_n
);

  // ========= Direcciones de registros =========
  localparam byte ADDR_CONTROL     = 8'h00;
  localparam byte ADDR_IN_W        = 8'h01;
  localparam byte ADDR_IN_H        = 8'h02;
  localparam byte ADDR_SCALE_Q88   = 8'h03;

  localparam byte ADDR_OUT_W       = 8'h04;
  localparam byte ADDR_OUT_H       = 8'h05;

  localparam byte ADDR_STATUS      = 8'h10;
  localparam byte ADDR_PERF_FLOPS  = 8'h11;
  localparam byte ADDR_PERF_MEM_RD = 8'h12;
  localparam byte ADDR_PERF_MEM_WR = 8'h13;
  localparam byte ADDR_PROGRESS    = 8'h14;

  localparam byte ADDR_IN_ADDR     = 8'h20;
  localparam byte ADDR_IN_DATA     = 8'h21;

  localparam byte ADDR_OUT_ADDR    = 8'h30;
  localparam byte ADDR_OUT_DATA    = 8'h31;

  localparam byte ADDR_IN_WADDR    = 8'h22;
  localparam byte ADDR_IN_WDATA    = 8'h23;

  localparam logic [31:0] DEFAULT_IN_W       = 32'd64;
  localparam logic [31:0] DEFAULT_IN_H       = 32'd64;
  localparam logic [15:0] DEFAULT_SCALE_Q88  = 16'd205;
  localparam logic [3:0]  START_PULSE_CYCLES = 4'd8;

  // ========= Dominio JTAG (tck) =========
  logic [DRW-1:0] dr_shift;
  logic [7:0]     latched_addr;
  logic [31:0]    dr_read_data;

  logic is_write, is_read;

  always_comb begin
    is_write = (ir_in == 2'b01);
    is_read  = (ir_in == 2'b10);
  end

  // Captura / desplazamiento de DR
  always_ff @(posedge tck or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      dr_shift     <= '0;
      latched_addr <= 8'd0;
    end else begin
      if (vs_cdr) begin
        dr_shift <= is_read ? {dr_read_data, latched_addr} : '0;
      end else if (vs_sdr) begin
        dr_shift <= {tdi, dr_shift[DRW-1:1]};
      end

      if (vs_udr && is_read) begin
        latched_addr <= dr_shift[7:0];
      end
    end
  end

  always_comb begin
    tdo = vs_sdr ? dr_shift[0] : 1'b0;
  end

  // UPDATE-DR en tck para escrituras
  logic        wr_pulse_tck;
  logic [7:0]  wr_addr_hold_tck;
  logic [31:0] wr_data_hold_tck;

  always_ff @(posedge tck or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      wr_pulse_tck     <= 1'b0;
      wr_addr_hold_tck <= 8'd0;
      wr_data_hold_tck <= 32'd0;
    end else begin
      wr_pulse_tck <= 1'b0;
      if (vs_udr && is_write) begin
        wr_pulse_tck     <= 1'b1;
        wr_addr_hold_tck <= dr_shift[7:0];
        wr_data_hold_tck <= dr_shift[39:8];
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

  logic wr_sys;
  assign wr_sys = (wr_tog_sync ^ wr_tog_sync_d);

  logic [31:0] wr_addr_sync1, wr_addr_sync2;
  logic [31:0] wr_data_sync1, wr_data_sync2;

  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      wr_addr_sync1 <= 32'd0;
      wr_addr_sync2 <= 32'd0;
      wr_data_sync1 <= 32'd0;
      wr_data_sync2 <= 32'd0;
    end else begin
      wr_addr_sync1 <= {24'd0, wr_addr_hold_tck};
      wr_addr_sync2 <= wr_addr_sync1;
      wr_data_sync1 <= wr_data_hold_tck;
      wr_data_sync2 <= wr_data_sync1;
    end
  end

  // ========= Lado sistema (clk_sys) =========
  // Registros de configuración y estado mínimo
  logic [31:0] reg_in_w, reg_in_h, reg_scale;
  logic        reg_mode_simd;

  logic [31:0] reg_in_raddr_pix;
  logic [31:0] reg_in_waddr_pix;
  logic [31:0] reg_out_raddr;

  logic [7:0]  reg_in_data_sys;
  logic [7:0]  reg_out_data_sys;

  logic [3:0]  start_cnt;
  assign start_pulse = (start_cnt != 4'd0);

  // FSM acceso memoria de entrada
  typedef enum logic [1:0] {
    M_IDLE,
    M_REQ_RD,
    M_WAIT_RD,
    M_REQ_WR
  } mem_fsm_t;

  mem_fsm_t   mem_fsm;
  logic       mem_is_write;
  logic [31:0] pix_addr_pending;
  logic [1:0]  lane_pending;
  logic [7:0]  byte_pending;
  logic [31:0] word_data_pending;

  assign out_mem_raddr = reg_out_raddr[AW-1:0];

  // Señales cableadas directas (sin registros intermedios)
  wire [31:0] w_status      = {29'd0, 1'b0, status_busy, status_done};
  wire [31:0] w_perf_flops  = perf_flops;
  wire [31:0] w_perf_mem_rd = perf_mem_rd;
  wire [31:0] w_perf_mem_wr = perf_mem_wr;
  wire [31:0] w_progress    = perf_mem_wr;
  wire [31:0] w_out_w       = {16'd0, out_w};
  wire [31:0] w_out_h       = {16'd0, out_h};

  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      reg_in_w         <= DEFAULT_IN_W;
      reg_in_h         <= DEFAULT_IN_H;
      reg_scale        <= {16'd0, DEFAULT_SCALE_Q88};

      reg_mode_simd    <= 1'b0;

      reg_in_raddr_pix <= 32'd0;
      reg_in_waddr_pix <= 32'd0;
      reg_out_raddr    <= 32'd0;

      reg_in_data_sys  <= 8'd0;
      reg_out_data_sys <= 8'd0;

      start_cnt        <= 4'd0;

      mem_fsm          <= M_IDLE;
      mem_is_write     <= 1'b0;
      pix_addr_pending <= 32'd0;
      lane_pending     <= 2'd0;
      byte_pending     <= 8'd0;
      word_data_pending<= 32'd0;

      mem_req_valid    <= 1'b0;
      mem_req_we       <= 1'b0;
      mem_req_addr     <= '0;
      mem_req_wdata    <= 32'd0;
    end else begin
      mem_req_valid <= 1'b0;
      mem_req_we    <= 1'b0;

      if (start_cnt != 4'd0)
        start_cnt <= start_cnt - 4'd1;

      // muestreo continuo de mem_out (8 bits)
      reg_out_data_sys <= out_mem_rdata;

      // Escrituras desde JTAG (un pulso wr_sys)
      if (wr_sys) begin
        unique case (wr_addr_sync2[7:0])
          ADDR_IN_W:        reg_in_w      <= wr_data_sync2;
          ADDR_IN_H:        reg_in_h      <= wr_data_sync2;
          ADDR_SCALE_Q88:   reg_scale     <= wr_data_sync2;

          ADDR_CONTROL: begin
            if (wr_data_sync2[0])
              start_cnt <= START_PULSE_CYCLES;
            reg_mode_simd <= wr_data_sync2[1];
          end

          ADDR_IN_ADDR: begin
            reg_in_raddr_pix <= wr_data_sync2;
            if (mem_fsm == M_IDLE) begin
              pix_addr_pending <= wr_data_sync2;
              lane_pending     <= wr_data_sync2[1:0];
              mem_is_write     <= 1'b0;
              mem_fsm          <= M_REQ_RD;
            end
          end

          ADDR_OUT_ADDR: begin
            reg_out_raddr <= wr_data_sync2;
          end

          ADDR_IN_WADDR: begin
            reg_in_waddr_pix <= wr_data_sync2;
          end

          ADDR_IN_WDATA: begin
            if (mem_fsm == M_IDLE) begin
              pix_addr_pending  <= reg_in_waddr_pix;
              lane_pending      <= reg_in_waddr_pix[1:0];
              byte_pending      <= wr_data_sync2[7:0];
              mem_is_write      <= 1'b1;
              mem_fsm           <= M_REQ_RD;
              reg_in_waddr_pix  <= reg_in_waddr_pix + 32'd1;
            end
          end

          default: ;
        endcase
      end

      // FSM de acceso a memoria de entrada
      unique case (mem_fsm)
        M_IDLE: begin
        end

        M_REQ_RD: begin
          if (!status_busy) begin
            mem_req_valid <= 1'b1;
            mem_req_we    <= 1'b0;
            mem_req_addr  <= pix_addr_pending[AW+1:2];
            mem_fsm       <= M_WAIT_RD;
          end
        end

        M_WAIT_RD: begin
          if (mem_resp_valid) begin
            word_data_pending <= mem_resp_rdata;
            unique case (lane_pending)
              2'd0: reg_in_data_sys <= mem_resp_rdata[7:0];
              2'd1: reg_in_data_sys <= mem_resp_rdata[15:8];
              2'd2: reg_in_data_sys <= mem_resp_rdata[23:16];
              default: reg_in_data_sys <= mem_resp_rdata[31:24];
            endcase
            if (mem_is_write)
              mem_fsm <= M_REQ_WR;
            else
              mem_fsm <= M_IDLE;
          end
        end

        M_REQ_WR: begin
          if (!status_busy) begin
            mem_req_valid <= 1'b1;
            mem_req_we    <= 1'b1;
            mem_req_addr  <= pix_addr_pending[AW+1:2];
            unique case (lane_pending)
              2'd0: mem_req_wdata <= {word_data_pending[31:8],  byte_pending};
              2'd1: mem_req_wdata <= {word_data_pending[31:16], byte_pending, word_data_pending[7:0]};
              2'd2: mem_req_wdata <= {word_data_pending[31:24], byte_pending, word_data_pending[15:0]};
              default: mem_req_wdata <= {byte_pending, word_data_pending[23:0]};
            endcase
            mem_fsm <= M_IDLE;
          end
        end

        default: mem_fsm <= M_IDLE;
      endcase
    end
  end

  // Exportar configuración al core/top
  assign cfg_in_w      = reg_in_w[15:0];
  assign cfg_in_h      = reg_in_h[15:0];
  assign cfg_scale_q88 = reg_scale[15:0];
  assign cfg_mode_simd = reg_mode_simd;

  // ========= CDC clk_sys -> tck para IN/OUT_DATA =========
  logic [7:0] in_data_meta,  in_data_sync;
  logic [7:0] out_data_meta, out_data_sync;

  always_ff @(posedge tck or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      in_data_meta  <= 8'd0;
      in_data_sync  <= 8'd0;
      out_data_meta <= 8'd0;
      out_data_sync <= 8'd0;
    end else begin
      in_data_meta  <= reg_in_data_sys;
      in_data_sync  <= in_data_meta;
      out_data_meta <= reg_out_data_sys;
      out_data_sync <= out_data_meta;
    end
  end

  // Multiplexor de lectura (lado JTAG) con señales cableadas
  always_comb begin
    unique case (latched_addr)
      ADDR_IN_W:        dr_read_data = reg_in_w;
      ADDR_IN_H:        dr_read_data = reg_in_h;
      ADDR_SCALE_Q88:   dr_read_data = reg_scale;

      ADDR_OUT_W:       dr_read_data = w_out_w;
      ADDR_OUT_H:       dr_read_data = w_out_h;

      ADDR_STATUS:      dr_read_data = w_status;

      ADDR_CONTROL:     dr_read_data = {30'd0, reg_mode_simd, 1'b0};

      ADDR_PERF_FLOPS:  dr_read_data = w_perf_flops;
      ADDR_PERF_MEM_RD: dr_read_data = w_perf_mem_rd;
      ADDR_PERF_MEM_WR: dr_read_data = w_perf_mem_wr;
      ADDR_PROGRESS:    dr_read_data = w_progress;

      ADDR_IN_DATA:     dr_read_data = {24'h0, in_data_sync};
      ADDR_OUT_DATA:    dr_read_data = {24'h0, out_data_sync};

      default:          dr_read_data = 32'hDEAD_BEEF;
    endcase
  end

endmodule
