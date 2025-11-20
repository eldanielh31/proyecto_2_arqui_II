// ref_bilinear_q88.cpp
// Modelo de referencia (C++) de la interpolación bilineal en Q8.8.
// Uso:
//   ref_bilinear_q88 <in_hex> <W> <H> <scale_q88> <out_hex>
//
// Donde:
//  - <in_hex>: archivo .hex (1 byte por línea, 2 dígitos hex, sin "0x")
//  - W, H: dimensiones de entrada
//  - scale_q88: entero Q8.8 (0.5..1.0 => 128..256)
//  - out_hex: archivo de salida .hex (1 byte por línea)
//
// Coincide con el hardware: clamp de bordes, pesos Q0.16, redondeo +0x8000 y >>16.

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>

static inline uint8_t sat_u8(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "Uso: " << argv[0] << " <in_hex> <W> <H> <scale_q88> <out_hex>\n";
    return 1;
  }
  std::string in_file  = argv[1];
  int W = std::atoi(argv[2]);
  int H = std::atoi(argv[3]);
  int scale_q88 = std::atoi(argv[4]);
  std::string out_file = argv[5];

  // Leer input hex
  std::ifstream fin(in_file);
  if (!fin) { std::cerr << "No se pudo abrir " << in_file << "\n"; return 1; }
  std::vector<uint8_t> img_in;
  img_in.reserve(W*H);
  std::string line;
  while (std::getline(fin, line)) {
    if (line.empty()) continue;
    unsigned v;
    std::stringstream ss; ss << std::hex << line;
    ss >> v;
    img_in.push_back((uint8_t)(v & 0xFF));
  }
  fin.close();
  if ((int)img_in.size() < W*H) {
    std::cerr << "Advertencia: el archivo tiene menos de W*H bytes (" << img_in.size() << ")\n";
  }

  auto mul_q88_floor = [](int v, int s)->int { return (v * s) >> 8; };
  int Wp = mul_q88_floor(W, scale_q88);
  int Hp = mul_q88_floor(H, scale_q88);

  std::vector<uint8_t> img_out((size_t)Wp * (size_t)Hp, 0);

  // inv_scale ~ 1/scale en Q8.8
  int inv_scale_q88;
  if (scale_q88 <= 0) inv_scale_q88 = 0xFFFF;
  else {
    int num = 65536 + (scale_q88 >> 8);
    inv_scale_q88 = num / scale_q88;
  }

  auto addr_of = [&](int x, int y)->int { return y*W + x; };

  int sy_fix = 0; // Q8.8
  for (int y = 0; y < Hp; ++y) {
    int sx_fix = 0; // Q8.8
    for (int x = 0; x < Wp; ++x) {
      int sx_int = (sx_fix >> 8);
      int sy_int = (sy_fix >> 8);
      int ax_q   = (sx_fix & 0xFF);
      int ay_q   = (sy_fix & 0xFF);

      int xi = sx_int;
      int yi = sy_int;
      int fx = ax_q;
      int fy = ay_q;

      if (sx_int >= W - 1) { xi = W - 2; fx = 255; }
      if (sy_int >= H - 1) { yi = H - 2; fy = 255; }

      int I00 = img_in[addr_of(xi,     yi    )];
      int I10 = img_in[addr_of(xi + 1, yi    )];
      int I01 = img_in[addr_of(xi,     yi + 1)];
      int I11 = img_in[addr_of(xi + 1, yi + 1)];

      int wx0 = 256 - fx; // Q0.8
      int wx1 = fx;
      int wy0 = 256 - fy;
      int wy1 = fy;

      int w00 = wx0 * wy0; // Q0.16
      int w10 = wx1 * wy0;
      int w01 = wx0 * wy1;
      int w11 = wx1 * wy1;

      uint32_t p00 = (uint32_t)w00 * (uint32_t)I00;
      uint32_t p10 = (uint32_t)w10 * (uint32_t)I10;
      uint32_t p01 = (uint32_t)w01 * (uint32_t)I01;
      uint32_t p11 = (uint32_t)w11 * (uint32_t)I11;

      uint32_t sum = p00 + p10 + p01 + p11;          // Q0.16 * u8 -> hasta 32b
      sum += 0x00008000;                             // +0.5
      uint8_t pix = (uint8_t)((sum >> 16) & 0xFF);   // >>16

      img_out[(size_t)y*(size_t)Wp + (size_t)x] = pix;

      // avanzar X
      sx_fix += inv_scale_q88;
    }
    // avanzar Y
    sy_fix += inv_scale_q88;
  }

  // Escribir salida hex
  std::ofstream fout(out_file);
  if (!fout) { std::cerr << "No se pudo abrir " << out_file << "\n"; return 1; }
  fout << std::hex << std::setfill('0');
  for (size_t i = 0; i < img_out.size(); ++i) {
    fout << std::setw(2) << (unsigned)img_out[i] << "\n";
  }
  fout.close();

  std::cerr << "OK. W'=" << Wp << " H'=" << Hp << " bytes=" << (Wp*Hp) << "\n";
  return 0;
}
