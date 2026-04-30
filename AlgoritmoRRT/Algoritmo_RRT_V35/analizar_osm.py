# analizar_osm.py
import sys
import xml.etree.ElementTree as ET
from shapely.geometry import box, LineString, MultiPolygon, Polygon
from shapely.ops import unary_union
from PIL import Image, ImageDraw
import numpy as np
from skimage.morphology import skeletonize

def procesar_mapa_calles(ruta_archivo):
    print(f"Python: Analizando red de calles en {ruta_archivo}...")

    try:
        tree = ET.parse(ruta_archivo)
        root = tree.getroot()
    except Exception as e:
        print(f"Error al leer el archivo XML: {e}")
        return

    nodos = {}
    nodos_usados_lon = []
    nodos_usados_lat = []
    for node in root.findall('node'):
        lon, lat = float(node.attrib['lon']), float(node.attrib['lat'])
        nodos[node.attrib['id']] = (lon, lat)

    lineas_crudas = []
    for way in root.findall('way'):
        es_calle = False
        for tag in way.findall('tag'):
            if tag.attrib.get('k') == 'highway':
                es_calle = True
                break

        if es_calle:
            nd_refs = [nd.attrib['ref'] for nd in way.findall('nd')]
            coords = []
            for ref in nd_refs:
                if ref in nodos:
                    lon, lat = nodos[ref]
                    coords.append((lon, lat))
                    nodos_usados_lon.append(lon)
                    nodos_usados_lat.append(lat)

            if len(coords) > 1:
                lineas_crudas.append(coords)

    if not lineas_crudas:
        print("Python: No se encontraron calles (highway) en el archivo.")
        return

    min_lon, max_lon = min(nodos_usados_lon), max(nodos_usados_lon)
    min_lat, max_lat = min(nodos_usados_lat), max(nodos_usados_lat)
    rango_lon = max_lon - min_lon if max_lon != min_lon else 0.0001
    rango_lat = max_lat - min_lat if max_lat != min_lat else 0.0001
    LADO_MAPA = 500.0

    lineas_escaladas = []
    for linea in lineas_crudas:
        puntos = []
        for lon, lat in linea:
            x = ((lon - min_lon) / rango_lon) * LADO_MAPA
            y = ((lat - min_lat) / rango_lat) * LADO_MAPA
            puntos.append((x, y))
        lineas_escaladas.append(LineString(puntos))

    print("Python: Calculando intersecciones espaciales (Manzanas)...")
    calles_infladas = [linea.buffer(3.5, cap_style=2, join_style=2) for linea in lineas_escaladas]
    red_de_calles = unary_union(calles_infladas)

    bloque_solido = box(0, 0, LADO_MAPA, LADO_MAPA)
    manzanas_crudas = bloque_solido.difference(red_de_calles)

    if isinstance(manzanas_crudas, Polygon):
        lista_manzanas = [manzanas_crudas]
    elif isinstance(manzanas_crudas, MultiPolygon):
        lista_manzanas = list(manzanas_crudas.geoms)
    else:
        lista_manzanas = []

    with open('Mapa_Obstaculos_RRT.m', 'w') as f:
        f.write("% Archivo generado vía Python (Espacio Negativo de Calles)\n")
        f.write("obstaculos = {\n")
        contador = 0
        for manzana in lista_manzanas:
            coords = list(manzana.exterior.coords)
            if len(coords) > 3:
                f.write("    [")
                v_strs = [f"{round(x, 2)}, {round(y, 2)}" for x, y in coords]
                f.write("; ".join(v_strs))
                f.write("];\n")
                contador += 1
        f.write("};\n")

    print(f"Python: ¡Éxito! Se generaron {contador} manzanas como obstáculos.")

    # =========================================================================
    # === GENERACIÓN DEL MAPA DISCRETO (GRID CACHING) ===
    # =========================================================================
    print("Python: Generando matriz de ocupación discreta (501x501)...")

    # Creamos un lienzo negro (0 = Calle libre) de tamaño 501x501
    img = Image.new('L', (int(LADO_MAPA) + 1, int(LADO_MAPA) + 1), 0)
    draw = ImageDraw.Draw(img)

    for manzana in lista_manzanas:
        coords = [(x, y) for x, y in manzana.exterior.coords]
        # Dibujamos el polígono en blanco (1 = Obstáculo / Edificio)
        draw.polygon(coords, fill=1)

    # Convertimos la imagen a matriz y la guardamos
    matriz_ocupacion = np.array(img)
    np.savetxt('Mapa_Discreto.txt', matriz_ocupacion, fmt='%d')
    print("Python: ¡Grid Caching guardado como 'Mapa_Discreto.txt'!")

    # =========================================================================
    # === EJE MEDIAL (SKELETONIZATION) ===
    # =========================================================================
    print("Python: Calculando el esqueleto de las calles...")

    # Las calles son 0. Creamos una matriz booleana donde True = Calle Libre
    matriz_calles = (matriz_ocupacion == 0)

    # Skeletonize colapsa las calles hasta dejar una línea de 1 pixel centrada
    esqueleto = skeletonize(matriz_calles)

    # Convertimos el booleano a enteros (1 = Centro de calle, 0 = Resto)
    matriz_esqueleto = np.uint8(esqueleto)
    np.savetxt('Mapa_Esqueleto.txt', matriz_esqueleto, fmt='%d')
    print("Python: ¡Eje Medial guardado como 'Mapa_Esqueleto.txt'!")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        procesar_mapa_calles(sys.argv[1])
    else:
        print("Error: No se proporcionó la ruta del archivo.")
