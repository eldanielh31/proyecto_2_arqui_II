import math
from PIL import Image
import os

def reconstruct_hex_to_image(hex_file_path, output_image_path):
    print(f"--- Procesando: {hex_file_path} ---")
    
    try:
        with open(hex_file_path, 'r') as f:
            content = f.read()
            
        # Limpiamos y parseamos los datos
        # Asumimos que los datos pueden venir separados por espacios o saltos de linea
        # Si vienen todos pegados (ej: AABBCC), habría que cambiar la lógica de lectura.
        hex_tokens = content.split()
        
        pixel_data = []
        for token in hex_tokens:
            try:
                # Convertir hex a int (base 16)
                val = int(token, 16)
                pixel_data.append(val)
            except ValueError:
                print(f"Advertencia: Se ignoró el token no válido '{token}'")

        total_pixels = len(pixel_data)
        
        if total_pixels == 0:
            print("Error: No se encontraron datos en el archivo.")
            return

        # Calcular dimensiones (N x N)
        root = math.sqrt(total_pixels)
        
        if int(root + 0.5) ** 2 != total_pixels:
            print(f"Error: La cantidad de datos ({total_pixels}) no forma un cuadrado perfecto.")
            print("Verifica que el archivo .hex esté completo o que la imagen sea cuadrada.")
            return
            
        n = int(root)
        print(f"Dimensiones detectadas: {n}x{n} ({total_pixels} píxeles)")

        # Crear la imagen en modo 'L' (Escala de grises, 8-bit)
        img = Image.new('L', (n, n))
        
        # Cargar los datos en la imagen
        img.putdata(pixel_data)
        
        # Guardar resultado
        img.save(output_image_path)
        print(f"¡Éxito! Imagen reconstruida guardada en: {output_image_path}")
        
        # Opcional: Mostrar la imagen inmediatamente
        img.show()

    except FileNotFoundError:
        print("Error: El archivo .hex no existe.")
    except Exception as e:
        print(f"Ocurrió un error inesperado: {e}")

# --- EJECUCIÓN ---
if __name__ == "__main__":
    # Cambia 'imagen_input.hex' por el nombre de tu archivo real
    input_file = "img_out_tb.hex"
    output_file = "imagen_reconstruida.png"
    
    reconstruct_hex_to_image(input_file, output_file)