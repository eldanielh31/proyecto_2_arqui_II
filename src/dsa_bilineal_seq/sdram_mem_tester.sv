`timescale 1ns / 1ps

// ============================================================================
// sdram_mem_tester.sv
//   - Tester sencillo para interfaz wide de 32 bits sobre SDRAM
//   - Escribe TEST_DEPTH palabras con un patrón, luego las lee y verifica.
//   - Expone flags de estado: test_running, test_pass, test_fail.
// ============================================================================

module sdram_mem_tester #(
  parameter int AW          = 16,    // ancho de dirección (32 bits por dirección)
  parameter int TEST_DEPTH  = 256    // número de palabras de 32 bits a probar
)(
  input  logic        clk,
  input  logic        rst_n,

  // Interfaz tipo wide_onchip_mem / sdram_wide_if
  output logic              req_valid,
  input  logic              req_ready,
  output logic              req_we,       // 1 = write, 0 = read
  output logic [AW-1:0]     req_addr,
  output logic [31:0]       req_wdata,
  input  logic              resp_valid,
  input  logic [31:0]       resp_rdata,

  // Estado del test
  output logic              test_running,
  output logic              test_pass,
  output logic              test_fail
);

  // Patrón de datos a escribir en función de la dirección
  function automatic [31:0] pattern(input logic [AW-1:0] a);
    pattern = {a, ~a};  // mitad baja = addr, mitad alta = complemento
  endfunction

  typedef enum logic [2:0] {
    ST_INIT_WAIT,   // espera inicial para que la SDRAM se inicialice
    ST_WRITE_REQ,   // emite peticiones de escritura
    ST_READ_REQ,    // emite peticiones de lectura
    ST_READ_WAIT,   // espera la respuesta y verifica
    ST_PASS,        // test ok
    ST_FAIL         // test con error
  } state_t;

  state_t        state;
  logic [AW-1:0] addr;           // dirección actual de prueba
  logic [19:0]   init_cnt;       // contador de espera inicial (~1M ciclos)
  logic          req_valid_r;
  logic          req_we_r;
  logic [AW-1:0] req_addr_r;
  logic [31:0]   req_wdata_r;

  logic          test_running_r;
  logic          test_pass_r;
  logic          test_fail_r;

  // Asignación de registros a salidas
  assign req_valid     = req_valid_r;
  assign req_we        = req_we_r;
  assign req_addr      = req_addr_r;
  assign req_wdata     = req_wdata_r;

  assign test_running  = test_running_r;
  assign test_pass     = test_pass_r;
  assign test_fail     = test_fail_r;

  // Lógica secuencial principal
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state          <= ST_INIT_WAIT;
      init_cnt       <= '0;
      addr           <= '0;
      req_valid_r    <= 1'b0;
      req_we_r       <= 1'b0;
      req_addr_r     <= '0;
      req_wdata_r    <= '0;
      test_running_r <= 1'b0;
      test_pass_r    <= 1'b0;
      test_fail_r    <= 1'b0;
    end else begin
      // Valores por defecto de flags
      test_running_r <= 1'b0;
      test_pass_r    <= 1'b0;
      test_fail_r    <= 1'b0;

      case (state)
        // ----------------------------------------------------------
        // Espera inicial (por ejemplo ~20 ms a 50 MHz, 1M ciclos)
        // ----------------------------------------------------------
        ST_INIT_WAIT: begin
          test_running_r <= 1'b1;
          req_valid_r    <= 1'b0;

          if (init_cnt != 20'd1_000_000) begin
            init_cnt <= init_cnt + 1'b1;
          end else begin
            init_cnt <= init_cnt;
            addr     <= '0;
            state    <= ST_WRITE_REQ;
          end
        end

        // ----------------------------------------------------------
        // Fase de escritura: envía TEST_DEPTH escrituras
        // ----------------------------------------------------------
        ST_WRITE_REQ: begin
          test_running_r <= 1'b1;

          // Si no hay petición activa, lanzar una nueva
          if (!req_valid_r) begin
            req_we_r    <= 1'b1;           // escritura
            req_addr_r  <= addr;
            req_wdata_r <= pattern(addr);
            req_valid_r <= 1'b1;
          end else begin
            // Espera a que el wrapper acepte la petición
            if (req_ready) begin
              req_valid_r <= 1'b0;
              // Siguiente dirección o cambio de fase
              if (addr == TEST_DEPTH-1) begin
                addr  <= '0;
                state <= ST_READ_REQ;
              end else begin
                addr  <= addr + 1'b1;
              end
            end
          end
        end

        // ----------------------------------------------------------
        // Fase de lectura: emite petición de lectura para addr actual
        // ----------------------------------------------------------
        ST_READ_REQ: begin
          test_running_r <= 1'b1;

          if (!req_valid_r) begin
            req_we_r    <= 1'b0;           // lectura
            req_addr_r  <= addr;
            req_wdata_r <= 32'h0000_0000; // ignorado en lectura
            req_valid_r <= 1'b1;
          end else begin
            if (req_ready) begin
              req_valid_r <= 1'b0;
              state       <= ST_READ_WAIT;
            end
          end
        end

        // ----------------------------------------------------------
        // Espera el dato leído y lo compara con el patrón
        // ----------------------------------------------------------
        ST_READ_WAIT: begin
          test_running_r <= 1'b1;

          if (resp_valid) begin
            if (resp_rdata !== pattern(addr)) begin
              // Mismatch → fracaso
              state       <= ST_FAIL;
            end else begin
              // Coincide → avanzar
              if (addr == TEST_DEPTH-1) begin
                state <= ST_PASS;
              end else begin
                addr  <= addr + 1'b1;
                state <= ST_READ_REQ;
              end
            end
          end
        end

        // ----------------------------------------------------------
        // Estados finales
        // ----------------------------------------------------------
        ST_PASS: begin
          test_pass_r    <= 1'b1;
          test_running_r <= 1'b0;
          // Se mantiene en PASS hasta reset
        end

        ST_FAIL: begin
          test_fail_r    <= 1'b1;
          test_running_r <= 1'b0;
          // Se mantiene en FAIL hasta reset
        end

        default: begin
          state <= ST_INIT_WAIT;
        end
      endcase
    end
  end

endmodule
