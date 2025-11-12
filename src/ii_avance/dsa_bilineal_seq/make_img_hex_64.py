# make_img_hex_64.py
# Requiere: Pillow  ->  pip install pillow

import os
import math
from PIL import Image  # type: ignore

# ---------------- Parámetros de entrada/salida ----------------
W_IN, H_IN = 64, 64
SCALE_Q88 = 205  # 0.80 en Q8.8
HEX_IN = "img_in_64x64.hex"
PNG_IN = "img_in_64x64.png"

HEX_OUT = "img_out.hex"
PNG_OUT = "img_out.png"

def q88_to_float(q):
    return q / 256.0

def compute_out_dims(w_in, h_in, scale_q88):
    s = q88_to_float(scale_q88)
    # out = floor(in * scale)
    return int(math.floor(w_in * s)), int(math.floor(h_in * s))

def generate_input_pattern(w, h):
    data = [[0 for _ in range(w)] for __ in range(h)]
    denom = (w - 1 + h - 1) if (w + h) > 2 else 1
    for y in range(h):
        for x in range(w):
            grad = int(((x + y) / denom) * 255)
            on_main = (x == y)
            on_anti = (x + y == w - 1)
            on_border = (x == 0 or x == w - 1 or y == 0 or y == h - 1)
            val = grad
            if on_main or on_anti or on_border:
                val = max(val, 220)
            data[y][x] = 0 if val < 0 else (255 if val > 255 else val)
    return data

def save_hex_y_major(data, path):
    h = len(data)
    w = len(data[0]) if h else 0
    with open(path, "w", newline="\n") as fh:
        for y in range(h):
            for x in range(w):
                fh.write(f"{data[y][x]:02x}\n")

def save_png_from_array(data, path):
    h = len(data)
    w = len(data[0]) if h else 0
    img = Image.new("L", (w, h))
    img.putdata([data[y][x] for y in range(h) for x in range(w)])
    img.save(path)

def read_hex_lines(path):
    with open(path, "r") as fh:
        return [ln.strip() for ln in fh if ln.strip()]

def to_matrix(vals, w, h):
    it = iter(vals[: w * h])  # recorte estricto
    return [[next(it) for _ in range(w)] for __ in range(h)]

def main():
    # 1) Generar imagen de entrada (HEX + PNG)
    data_in = generate_input_pattern(W_IN, H_IN)
    save_hex_y_major(data_in, HEX_IN)
    save_png_from_array(data_in, PNG_IN)
    print(f"[OK] Generados: {HEX_IN} ({W_IN}x{H_IN}) y {PNG_IN}")

    # 2) Convertir salida (HEX -> PNG) recortando a out_w*out_h
    if os.path.exists(HEX_OUT):
        out_w, out_h = compute_out_dims(W_IN, H_IN, SCALE_Q88)  # 64 * 0.80 => 51
        lines = read_hex_lines(HEX_OUT)
        n = len(lines)
        need = out_w * out_h

        if n < need:
            raise ValueError(f"{HEX_OUT} tiene {n} bytes, se requieren {need} para {out_w}x{out_h}.")

        # Tomar solo los primeros out_w*out_h bytes (y-major), ignorando el resto de la RAM
        vals = [int(ln, 16) for ln in lines[:need]]
        data_out = to_matrix(vals, out_w, out_h)
        save_png_from_array(data_out, PNG_OUT)
        print(f"[OK] Convertido: {HEX_OUT} -> {PNG_OUT} ({out_w}x{out_h}), usando recorte a {need} bytes")
    else:
        print(f"[INFO] No se encontró {HEX_OUT}; se omitió la conversión de salida.")

if __name__ == "__main__":
    main()
