#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cstdint>
#include <iomanip>
#include <stdexcept>

// -----------------------------------------------------------------------------
// Lee un archivo .hex estilo $readmemh (bytes 8-bit) y los devuelve en un vector
// -----------------------------------------------------------------------------
std::vector<uint8_t> read_hex_image(const std::string &path) {
    std::ifstream fin(path.c_str());
    if (!fin) {
        throw std::runtime_error("No se pudo abrir el archivo de entrada: " + path);
    }

    std::vector<uint8_t> data;
    std::string token;
    while (fin >> token) {
        // Elimina posible prefijo "0x" o "0X"
        if (token.size() > 2 && (token[0] == '0') &&
            (token[1] == 'x' || token[1] == 'X')) {
            token = token.substr(2);
        }

        unsigned value = 0;
        std::stringstream ss;
        ss << std::hex << token;
        ss >> value;

        data.push_back(static_cast<uint8_t>(value & 0xFF));
    }

    return data;
}

// -----------------------------------------------------------------------------
// Escribe un vector de bytes a un archivo .hex (un byte por línea, 2 dígitos)
// -----------------------------------------------------------------------------
void write_hex_image(const std::string &path, const std::vector<uint8_t> &data) {
    std::ofstream fout(path.c_str());
    if (!fout) {
        throw std::runtime_error("No se pudo abrir el archivo de salida: " + path);
    }

    for (size_t i = 0; i < data.size(); ++i) {
        fout << std::hex << std::setfill('0') << std::setw(2)
             << static_cast<unsigned>(data[i]) << "\n";
    }
}

// -----------------------------------------------------------------------------
// Modelo de referencia: downscaling bilineal Q8.8 (exacto al descrito en HW)
//
// - in_w, in_h: dimensiones de entrada
// - scale_q88 : escala en Q8.8 (>0). p.ej: 0.8 -> 205
// - src       : imagen de entrada (tamaño >= in_w * in_h)
// - out_w/out_h: dimensiones de salida calculadas como en el HW
// -----------------------------------------------------------------------------
std::vector<uint8_t> bilinear_downscale_q88(
    const std::vector<uint8_t> &src,
    uint16_t in_w,
    uint16_t in_h,
    uint16_t scale_q88,
    uint16_t &out_w,
    uint16_t &out_h
) {
    if (in_w == 0 || in_h == 0) {
        throw std::runtime_error("Dimensiones de entrada inválidas (0).");
    }
    if (src.size() < static_cast<size_t>(in_w) * in_h) {
        throw std::runtime_error("El tamaño del buffer de entrada no coincide con in_w*in_h.");
    }

    // -------------------- Dimensiones de salida --------------------
    // out = floor((in * scale_q88) / 2^8)
    uint32_t mul_w = static_cast<uint32_t>(in_w) * static_cast<uint32_t>(scale_q88);
    uint32_t mul_h = static_cast<uint32_t>(in_h) * static_cast<uint32_t>(scale_q88);

    out_w = static_cast<uint16_t>((mul_w >> 8) & 0xFFFF);
    out_h = static_cast<uint16_t>((mul_h >> 8) & 0xFFFF);

    if (out_w == 0 || out_h == 0) {
        throw std::runtime_error("Dimensiones de salida resultan 0; revisar escala.");
    }

    std::vector<uint8_t> dst(static_cast<size_t>(out_w) * out_h, 0);

    // -------------------- Cálculo de inv_scale_q88 --------------------
    // inv_scale_q88 = floor(65536 / scale_q88) = 1/scale en Q8.8
    uint16_t inv_scale_q88;
    if (scale_q88 == 0) {
        inv_scale_q88 = 0xFFFF; // mismo caso especial que en HW
    } else {
        uint32_t num = 65536u; // 256 * 256
        inv_scale_q88 = static_cast<uint16_t>(num / scale_q88);
    }

    // En el HW: sx_fix, sy_fix son Q16.8 en 24 bits; aquí se usan 32 bits.
    // El incremento es {8'd0, inv_scale_q88}, que numéricamente es inv_scale_q88
    // interpretado en Q16.8.
    uint32_t sy_fix = 0; // Q16.8

    const uint32_t ROUND_Q016 = 0x00008000u; // para pasar de Q8.16 a entero con redondeo
    const uint32_t ONE_Q08    = 256u;        // 1.0 en Q0.8

    for (uint16_t oy = 0; oy < out_h; ++oy) {

        uint32_t sx_fix = 0; // Q16.8, reinicio por fila

        for (uint16_t ox = 0; ox < out_w; ++ox) {
            // ---------------- Coordenadas fuente desde sx_fix/sy_fix ----------------
            uint16_t sx_int = static_cast<uint16_t>(sx_fix >> 8);      // parte entera X
            uint8_t  ax_q   = static_cast<uint8_t>(sx_fix & 0xFFu);    // fracción X (Q0.8)

            uint16_t sy_int = static_cast<uint16_t>(sy_fix >> 8);      // parte entera Y
            uint8_t  ay_q   = static_cast<uint8_t>(sy_fix & 0xFFu);    // fracción Y (Q0.8)

            uint16_t xi_base = sx_int;
            uint16_t yi_base = sy_int;
            uint8_t  fx_q    = ax_q;
            uint8_t  fy_q    = ay_q;

            // ---------------- Clamping de borde (igual a HW) ----------------
            if (sx_int >= static_cast<uint16_t>(in_w - 1)) {
                xi_base = static_cast<uint16_t>(in_w - 2);
                fx_q    = 0xFF;
            }
            if (sy_int >= static_cast<uint16_t>(in_h - 1)) {
                yi_base = static_cast<uint16_t>(in_h - 2);
                fy_q    = 0xFF;
            }

            // ---------------- Lectura de vecinos I00, I10, I01, I11 ----------------
            // Conversión (x, y, width) -> dirección lineal
            auto linaddr = [in_w](uint16_t x, uint16_t y) -> uint32_t {
                return static_cast<uint32_t>(y) * static_cast<uint32_t>(in_w)
                     + static_cast<uint32_t>(x);
            };

            uint32_t addr00 = linaddr(xi_base,         yi_base);
            uint32_t addr10 = linaddr(xi_base + 1,     yi_base);
            uint32_t addr01 = linaddr(xi_base,         yi_base + 1);
            uint32_t addr11 = linaddr(xi_base + 1,     yi_base + 1);

            uint8_t I00 = src[addr00];
            uint8_t I10 = src[addr10];
            uint8_t I01 = src[addr01];
            uint8_t I11 = src[addr11];

            // ---------------- Pesos bilineales en Q0.8 / Q0.16 ----------------
            uint32_t wx0 = ONE_Q08 - static_cast<uint32_t>(fx_q);
            uint32_t wx1 = static_cast<uint32_t>(fx_q);
            uint32_t wy0 = ONE_Q08 - static_cast<uint32_t>(fy_q);
            uint32_t wy1 = static_cast<uint32_t>(fy_q);

            uint32_t w00 = wx0 * wy0; // Q0.16
            uint32_t w10 = wx1 * wy0;
            uint32_t w01 = wx0 * wy1;
            uint32_t w11 = wx1 * wy1;

            // ---------------- Productos w_ij * I_ij en Q8.16 ----------------
            uint32_t p00 = w00 * static_cast<uint32_t>(I00);
            uint32_t p10 = w10 * static_cast<uint32_t>(I10);
            uint32_t p01 = w01 * static_cast<uint32_t>(I01);
            uint32_t p11 = w11 * static_cast<uint32_t>(I11);

            uint32_t sum_q016 = p00 + p10 + p01 + p11;
            uint32_t sum_rounded = sum_q016 + ROUND_Q016;

            uint32_t pix_val = (sum_rounded >> 16) & 0xFFu;
            if (pix_val > 255u) pix_val = 255u; // por seguridad, aunque en teoría no debería pasar

            dst[static_cast<size_t>(oy) * out_w + ox] =
                static_cast<uint8_t>(pix_val);

            // ---------------- Avance en X (sx_fix_{n+1} = sx_fix_n + inv_scale_q88) ----
            sx_fix += static_cast<uint32_t>(inv_scale_q88);
        }

        // ---------------- Avance en Y (sy_fix_{m+1} = sy_fix_m + inv_scale_q88) -------
        sy_fix += static_cast<uint32_t>(inv_scale_q88);
    }

    return dst;
}

// -----------------------------------------------------------------------------
// main: wrapper sencillo para usar el modelo de referencia desde consola
// -----------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    try {
        if (argc != 6) {
            std::cerr
                << "Uso: " << argv[0]
                << " <input_hex> <in_w> <in_h> <scale_q88> <output_hex>\n"
                << "Ejemplo: " << argv[0]
                << " img_16x16.hex 16 16 205 img_out_ref.hex\n";
            return 1;
        }

        std::string input_path  = argv[1];
        uint16_t in_w           = static_cast<uint16_t>(std::stoul(argv[2], nullptr, 0));
        uint16_t in_h           = static_cast<uint16_t>(std::stoul(argv[3], nullptr, 0));
        uint16_t scale_q88      = static_cast<uint16_t>(std::stoul(argv[4], nullptr, 0));
        std::string output_path = argv[5];

        std::vector<uint8_t> src = read_hex_image(input_path);

        uint16_t out_w = 0, out_h = 0;
        std::vector<uint8_t> dst = bilinear_downscale_q88(
            src, in_w, in_h, scale_q88, out_w, out_h
        );

        write_hex_image(output_path, dst);

        std::cerr << "Procesamiento completado.\n";
        std::cerr << "Dimensiones salida: " << out_w << " x " << out_h << "\n";
        return 0;

    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
