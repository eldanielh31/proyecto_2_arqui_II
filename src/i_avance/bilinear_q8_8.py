#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Downscaling bilineal con aritmética fija Q8.8 (sin floats en el núcleo).
- Entrada: imagen real (se convertirá a escala de grises uint8)
- Salida: imagen reducida (uint8)
- Factor: --scale (0.50..1.00) o --scale-q (128..256), donde Q8.8 = round(scale*256)

Ejemplos:
  python3 bilinear_q8_8.py --input entrada.png --scale 0.75 --out salida.png
  python3 bilinear_q8_8.py --input entrada.jpg --scale-q 205 --out out.png
"""

import argparse
import os
from typing import Tuple
import numpy as np
from PIL import Image


def load_u8_grayscale(path: str) -> np.ndarray:
    """Carga una imagen y la retorna como arreglo uint8 (H,W) en grises."""
    img = Image.open(path).convert("L")  # forzar escala de grises
    arr = np.asarray(img, dtype=np.uint8)
    return arr


def save_u8_grayscale(arr: np.ndarray, path: str) -> None:
    """Guarda un arreglo uint8 (H,W) como imagen en escala de grises."""
    Image.fromarray(arr, mode="L").save(path)


def compute_output_shape(h_in: int, w_in: int, scale_q8_8: int) -> Tuple[int, int]:
    """Calcula tamaño de salida usando entero fijo Q8.8."""
    w_out = (w_in * scale_q8_8) >> 8
    h_out = (h_in * scale_q8_8) >> 8
    # Garantizar al menos 1x1
    w_out = max(1, int(w_out))
    h_out = max(1, int(h_out))
    return h_out, w_out


def downscale_bilinear_q8_8(img_u8: np.ndarray, scale_q8_8: int) -> np.ndarray:
    """
    Downscaling bilineal con aritmética fija:
    - Coordenadas fuente: usa recíproco de s en Q8.8 para evitar división en tiempo de ejecución.
    - Pesos: Q0.8 * Q0.8 -> Q0.16; acumulación y normalización (>>16).
    Todo el cómputo interno se realiza en enteros.
    """
    h_in, w_in = img_u8.shape
    if h_in < 2 or w_in < 2:
        raise ValueError("La imagen debe tener al menos 2x2 píxeles.")

    # Tamaño de salida
    h_out, w_out = compute_output_shape(h_in, w_in, scale_q8_8)
    out = np.zeros((h_out, w_out), dtype=np.uint8)

    # inv_s_q8_8 = round(256/s) = (256*256)/scale_q8_8, con redondeo entero
    inv_s_q8_8 = (256 * 256 + (scale_q8_8 // 2)) // scale_q8_8

    # Bucle principal
    for yo in range(h_out):
        # y_src_q en Q0.16
        y_src_q = yo * inv_s_q8_8
        yi = (y_src_q >> 8)  # parte entera
        fy = (y_src_q & 0xFF)  # fracción Q0.8

        # Evitar salir de rango al tomar yi+1
        if yi >= h_in - 1:
            yi = h_in - 2
            fy = 255  # máximo

        wy0 = 256 - fy
        wy1 = fy

        for xo in range(w_out):
            x_src_q = xo * inv_s_q8_8
            xi = (x_src_q >> 8)
            fx = (x_src_q & 0xFF)

            if xi >= w_in - 1:
                xi = w_in - 2
                fx = 255

            wx0 = 256 - fx
            wx1 = fx

            # Vecinos
            p00 = int(img_u8[yi,     xi    ])
            p10 = int(img_u8[yi,     xi + 1])
            p01 = int(img_u8[yi + 1, xi    ])
            p11 = int(img_u8[yi + 1, xi + 1])

            # Pesos bilineales en Q0.16
            w00 = wx0 * wy0
            w10 = wx1 * wy0
            w01 = wx0 * wy1
            w11 = wx1 * wy1

            # Acumulación (Q0.16 * u8) y normalización >>16
            acc = (w00 * p00) + (w10 * p10) + (w01 * p01) + (w11 * p11)
            val = acc >> 16
            if val > 255:
                val = 255
            out[yo, xo] = val

    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Downscaling bilineal fijo Q8.8 para imágenes en grises (8-bit).")
    p.add_argument("--input", required=True, help="Ruta de la imagen de entrada (PNG/JPG).")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--scale", type=float, help="Factor decimal en rango [0.50, 1.00].")
    g.add_argument("--scale-q", type=int, help="Factor en Q8.8 (128..256).")
    p.add_argument("--out", required=False, help="Ruta de salida (PNG). Si se omite, se genera automáticamente.")
    p.add_argument("--max-size", type=int, default=512, help="Tamaño máximo permitido en cada dimensión (por defecto 512).")
    return p.parse_args()


def main():
    args = parse_args()

    # Carga y validaciones
    img = load_u8_grayscale(args.input)
    h_in, w_in = img.shape
    if h_in > args.max_size or w_in > args.max_size:
        raise ValueError(f"La imagen ({w_in}x{h_in}) supera el máximo permitido {args.max_size}x{args.max_size}.")

    # Factor Q8.8
    if args.scale_q is not None:
        scale_q = int(args.scale_q)
        if not (128 <= scale_q <= 256):
            raise ValueError("scale-q debe estar entre 128 (0.50) y 256 (1.00).")
        scale_eff = scale_q / 256.0
    else:
        s = float(args.scale)
        if not (0.50 <= s <= 1.00):
            raise ValueError("scale (decimal) debe estar en [0.50, 1.00].")
        scale_q = int(round(s * 256.0))   # cuantización a Q8.8
        scale_eff = scale_q / 256.0

    h_out, w_out = compute_output_shape(h_in, w_in, scale_q)
    print(f"Entrada: {w_in}x{h_in} | scale_q8_8={scale_q} (~{scale_eff:.4f}) -> Salida: {w_out}x{h_out}")

    # Procesamiento
    out = downscale_bilinear_q8_8(img, scale_q)

    # Salida
    if args.out:
        out_path = args.out
    else:
        base, _ = os.path.splitext(os.path.basename(args.input))
        out_path = f"{base}_down_{int(round(scale_eff*100))}.png"

    save_u8_grayscale(out, out_path)
    print(f"Guardado: {out_path}")


if __name__ == "__main__":
    main()
