`timescale 1ns / 1ps

// ============================================================================
// sdram_wide_if.sv
//   - Interfaz de memoria "wide" (32 bits, tipo wide_onchip_mem) sobre SDRAM
//   - Cada dirección lógica (req_addr) representa una palabra de 32 bits
//   - Internamente se generan dos accesos de 16 bits (even/odd) a la SDRAM
//   - Implementa un handshake simple: una sola petición en vuelo
// ============================================================================

module sdram_wide_if #(
  parameter int AW = 16,                  // ancho de dirección de palabra de 32 bits
  parameter [24:0] BASE_ADDR = 25'd0      // offset lineal dentro de la SDRAM (en palabras de 16 bits)
)(
  input  logic        clk,
  input  logic        rst_n,

  // Interfaz tipo wide_onchip_mem (solo una petición en vuelo)
  input  logic              req_valid,
  output logic              req_ready,
  input  logic              req_we,       // 1 = write (32b), 0 = read (32b)
  input  logic [AW-1:0]     req_addr,     // dirección de palabra de 32 bits
  input  logic [31:0]       req_wdata,
  output logic              resp_valid,
  output logic [31:0]       resp_rdata,

  // Puertos físicos hacia la SDRAM externa (DE1-SoC / MTL2)
  output logic [12:0]       DRAM_ADDR,
  output logic  [1:0]       DRAM_BA,
  output logic              DRAM_CAS_N,
  output logic              DRAM_CKE,
  output logic              DRAM_CLK,
  output logic              DRAM_CS_N,
  inout  wire  [15:0]       DRAM_DQ,
  output logic  [1:0]       DRAM_DQM,
  output logic              DRAM_RAS_N,
  output logic              DRAM_WE_N
);

  // ==========================================================================
  // Reloj SDRAM: primera versión, se reenvía clk directamente
  // ==========================================================================
  assign DRAM_CLK = clk;

  // ==========================================================================
  // Interfaz interna tipo Avalon hacia DE1_SoC_QSYS_sdram (16 bits)
  // ==========================================================================
  logic [24:0] az_addr;
  logic [15:0] az_data;
  logic  [1:0] az_be_n;
  logic        az_cs;
  logic        az_rd_n;
  logic        az_wr_n;

  logic [15:0] za_data;
  logic        za_valid;
  logic        za_waitrequest;

  // Instancia del controlador SDRAM proporcionado por Terasic/Altera
  DE1_SoC_QSYS_sdram u_sdram (
    .az_addr        (az_addr),
    .az_be_n        (az_be_n),
    .az_cs          (az_cs),
    .az_data        (az_data),
    .az_rd_n        (az_rd_n),
    .az_wr_n        (az_wr_n),
    .clk            (clk),
    .reset_n        (rst_n),

    .za_data        (za_data),
    .za_valid       (za_valid),
    .za_waitrequest (za_waitrequest),

    .zs_addr        (DRAM_ADDR),
    .zs_ba          (DRAM_BA),
    .zs_cas_n       (DRAM_CAS_N),
    .zs_cke         (DRAM_CKE),
    .zs_cs_n        (DRAM_CS_N),
    .zs_dq          (DRAM_DQ),
    .zs_dqm         (DRAM_DQM),
    .zs_ras_n       (DRAM_RAS_N),
    .zs_we_n        (DRAM_WE_N)
  );

  // ==========================================================================
  // Registro de petición actual (una sola en vuelo)
  // ==========================================================================
  typedef enum logic [2:0] {
    ST_IDLE,
    ST_WRITE_LO,
    ST_WRITE_HI,
    ST_READ_LO_CMD,
    ST_READ_LO_WAIT,
    ST_READ_HI_CMD,
    ST_READ_HI_WAIT,
    ST_RESP
  } state_t;

  state_t               state;

  logic [AW-1:0]        req_addr_reg;
  logic [31:0]          req_wdata_reg;
  logic                 req_we_reg;

  logic [24:0]          addr_even_reg;
  logic [24:0]          addr_odd_reg;

  logic [15:0]          lo_data_reg;
  logic [31:0]          resp_data_reg;

  // ==========================================================================
  // Máquina de estados secuencial
  // ==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state         <= ST_IDLE;
      req_addr_reg  <= '0;
      req_wdata_reg <= '0;
      req_we_reg    <= 1'b0;
      addr_even_reg <= '0;
      addr_odd_reg  <= '0;
      lo_data_reg   <= 16'd0;
      resp_data_reg <= 32'd0;
    end else begin
      case (state)
        ST_IDLE: begin
          if (req_valid) begin
            req_addr_reg  <= req_addr;
            req_wdata_reg <= req_wdata;
            req_we_reg    <= req_we;

            // Cálculo de direcciones lineales de 16 bits:
            //   index = cero_ext(req_addr)    (25 bits)
            //   even  = BASE_ADDR + (index << 1)
            //   odd   = even + 1
            addr_even_reg <= BASE_ADDR + ( { {(25-AW){1'b0}}, req_addr } << 1 );
            addr_odd_reg  <= BASE_ADDR + ( { {(25-AW){1'b0}}, req_addr } << 1 ) + 25'd1;

            if (req_we)
              state <= ST_WRITE_LO;
            else
              state <= ST_READ_LO_CMD;
          end
        end

        // ---------------------- Escritura 32 bits -> 2 x 16 bits ----------------
        ST_WRITE_LO: begin
          // En este estado se mantiene el comando de escritura hasta que
          // za_waitrequest sea 0 (FIFO acepta la petición).
          if (!za_waitrequest) begin
            state <= ST_WRITE_HI;
          end
        end

        ST_WRITE_HI: begin
          if (!za_waitrequest) begin
            // Para este wrapper no se devuelve resp_valid en escrituras;
            // la operación termina aquí.
            state <= ST_IDLE;
          end
        end

        // ---------------------- Lectura 32 bits <- 2 x 16 bits ------------------
        ST_READ_LO_CMD: begin
          // En este estado se presenta el comando de lectura de la mitad baja
          // hasta que waitrequest sea 0.
          if (!za_waitrequest) begin
            state <= ST_READ_LO_WAIT;
          end
        end

        ST_READ_LO_WAIT: begin
          // Espera a que za_valid indique que el dato de 16 bits está listo
          if (za_valid) begin
            lo_data_reg <= za_data;       // mitad baja
            state       <= ST_READ_HI_CMD;
          end
        end

        ST_READ_HI_CMD: begin
          // Comando de lectura de la mitad alta
          if (!za_waitrequest) begin
            state <= ST_READ_HI_WAIT;
          end
        end

        ST_READ_HI_WAIT: begin
          if (za_valid) begin
            // Combina mitad alta y baja en un dato de 32 bits:
            // [31:16] = palabra alta, [15:0] = palabra baja
            resp_data_reg <= {za_data, lo_data_reg};
            state         <= ST_RESP;
          end
        end

        ST_RESP: begin
          // Un ciclo con resp_valid = 1
          state <= ST_IDLE;
        end

        default: begin
          state <= ST_IDLE;
        end
      endcase
    end
  end

  // ==========================================================================
  // Lógica combinacional de salidas (handshake + comandos SDRAM)
  // ==========================================================================
  always_comb begin
    // Valores por defecto
    req_ready   = 1'b0;
    resp_valid  = 1'b0;
    resp_rdata  = resp_data_reg;

    az_addr     = 25'd0;
    az_data     = 16'd0;
    az_be_n     = 2'b11;
    az_cs       = 1'b0;
    az_rd_n     = 1'b1;
    az_wr_n     = 1'b1;

    case (state)
      ST_IDLE: begin
        req_ready = 1'b1;
      end

      // Escrituras
      ST_WRITE_LO: begin
        az_addr = addr_even_reg;
        az_data = req_wdata_reg[15:0];
        az_be_n = 2'b00;
        az_cs   = 1'b1;
        az_wr_n = 1'b0;
      end

      ST_WRITE_HI: begin
        az_addr = addr_odd_reg;
        az_data = req_wdata_reg[31:16];
        az_be_n = 2'b00;
        az_cs   = 1'b1;
        az_wr_n = 1'b0;
      end

      // Lecturas
      ST_READ_LO_CMD: begin
        az_addr = addr_even_reg;
        az_be_n = 2'b00;
        az_cs   = 1'b1;
        az_rd_n = 1'b0;
      end

      ST_READ_LO_WAIT: begin
        // no se emiten nuevos comandos; se espera za_valid
      end

      ST_READ_HI_CMD: begin
        az_addr = addr_odd_reg;
        az_be_n = 2'b00;
        az_cs   = 1'b1;
        az_rd_n = 1'b0;
      end

      ST_READ_HI_WAIT: begin
        // no se emiten nuevos comandos; se espera za_valid
      end

      ST_RESP: begin
        resp_valid = 1'b1;
      end

      default: begin
        // mantiene valores por defecto
      end
    endcase
  end

endmodule
