# =========================================================================
# === FUNCION DE PREPROCESAMIENTO: ANÁLISIS DE MAPAS OSM ===
#
# Se extrae la información de un mapa OpenStreetMap (.osm), filtrando solo las calles. Posteriormente, se calcula el espacio negativo
# para generar los obstáculos y se exportan las matrices discretas para las colisiones y la generación de semillas
# =========================================================================
import sys                                      # módulo para recibir argumentos de la terminal
import xml.etree.ElementTree as ET              # módulo para leer archivos XML (.osm)
from shapely.geometry import box, LineString, MultiPolygon, Polygon # herramientas geométricas
from shapely.ops import unary_union             #  función para fusionar geometrías
from PIL import Image, ImageDraw                # herramientas para generar imágenes de matrices
import numpy as np                              # librería para el manejo de matrices matematicas
from skimage.morphology import skeletonize      # función para adelgazar las calles a 1 pixel

def procesar_mapa_calles(ruta_archivo):

    # --- LECTURA DEL ARCHIVO ---
    print(f"Python: Analizando red de calles en {ruta_archivo}...") # Se imprime un aviso de inicio.

    try:
        tree = ET.parse(ruta_archivo)                   # Analizar la estructura del archivo XML
        root = tree.getroot()                           # Obtener el elemento raíz de los datos
    except Exception as e:
        print(f"Error al leer el archivo XML: {e}")     # Muestra un error si el archivo está dañado
        return                                          # Se detiene la ejecución en caso de fallo

    # --- EXTRACCIÓN DE PUNTOS GEOGRÁFICOS ---
    nodos = {}                                          # Se crea un diccionario para almacenar las coordenadas
    nodos_usados_lon = []                               # Se inicia una lista para las longitudes (eje X) útiles
    nodos_usados_lat = []                               # Se inicia una lista para las latitudes (eje Y) útiles

    for node in root.findall('node'):                   # Se recorren todos los puntos del mapa
        lon, lat = float(node.attrib['lon']), float(node.attrib['lat']) # Extraer las coordenadas
        nodos[node.attrib['id']] = (lon, lat)           # Se asocian las coordenadas a su ID correspondiente

    # --- FILTRADO DE CALLES ---
    lineas_crudas = []                                  # Creación de contenedor para las rutas vehiculares validadas.

    for way in root.findall('way'):                     # Exploración secuencial de todas las vías trazadas
        es_calle = False                                # Apagador lógico de validación vial
        for tag in way.findall('tag'):                  # Inspección de la metadata de la vía actual
            if tag.attrib.get('k') == 'highway':        # Búsqueda de la etiqueta clasificatoria de calle
                es_calle = True                         # Confirmación de transito vehicular permitido
                break                                   # Ruptura del ciclo de etiquetas para optimización de recursos

        if es_calle:                                    # Condicionante de procesamiento exclusivo para calles
            nd_refs = [nd.attrib['ref'] for nd in way.findall('nd')] # Recopilación de los identificadores de cada tramo de la calle
            coords = []                                 # Lista temporal para el ensamble de la geometria
            for ref in nd_refs:                         # Recorrido de los puntos individuales del tramo
                if ref in nodos:                        # Comprobación de existencia del punto en la base de datos previa
                    lon, lat = nodos[ref]               # Recuperación de las coordenadas exactas
                    coords.append((lon, lat))           # Inserción del punto a la geometría de la ruta
                    nodos_usados_lon.append(lon)        # Documentación de la longitud para futuros límites de mapa
                    nodos_usados_lat.append(lat)        # Documentación de la latitud para límites de mapa

            if len(coords) > 1:                         # Validación de la calle (mínimo exigido: un punto inicial y uno final)
                lineas_crudas.append(coords)            # Almacenamiento definitivo de la vía estructurada

    if not lineas_crudas:                               # Chequeo de seguridad ante mapas carentes de calles utiles
        print("Python: No se encontraron calles en el archivo.") # Notificacion del error de contenido
        return # Terminación prematura del flujo.

    # --- 4. ESCALADO AL ENTORNO VIRTUAL (0 a 500) ---
    min_lon, max_lon = min(nodos_usados_lon), max(nodos_usados_lon) # Identificación de los límites horizontales absolutos.
    min_lat, max_lat = min(nodos_usados_lat), max(nodos_usados_lat) # Identificación de los límites verticales absolutos.
    rango_lon = max_lon - min_lon if max_lon != min_lon else 0.0001 # Cálculo de la anchura total (con prevención de división nula).
    rango_lat = max_lat - min_lat if max_lat != min_lat else 0.0001 # Cálculo de la altura total operativa.
    LADO_MAPA = 500.0                                               # Establecimiento de la constante de resolución gráfica de salida.

    lineas_escaladas = []                                           # Preparación del contenedor para las calles normalizadas.
    for linea in lineas_crudas:                                     # Procesamiento escalonado de las rutas originales.
        puntos = []                                                 # Lista temporal para las coordenadas proyectadas.
        for lon, lat in linea:                                      # Traducción individual de cada nodo topológico.
            x = ((lon - min_lon) / rango_lon) * LADO_MAPA           # Interpolación matemática hacia la escala horizontal 0-500.
            y = ((lat - min_lat) / rango_lat) * LADO_MAPA           # Interpolación matemática hacia la escala vertical 0-500.
            puntos.append((x, y))                                   # Integración de la coordenada escalada.
        lineas_escaladas.append(LineString(puntos))                 # Transformación del arreglo a un objeto vectorial.

    # --- 5. OBTENCIÓN DE EDIFICIOS (ESPACIO NEGATIVO) ---
    print("Python: Calculando intersecciones espaciales (Manzanas)...") # Reporte de avance en consola
    calles_infladas = [linea.buffer(3.5, cap_style=2, join_style=2) for linea in lineas_escaladas] # Expansión volumétrica de la línea para simular el ancho físico de una avenida
    red_de_calles = unary_union(calles_infladas)                        # Consolidación de todos los vectores en una macromalla de asfalto ininterrumpida

    bloque_solido = box(0, 0, LADO_MAPA, LADO_MAPA)                 # Generación de una placa geométrica sólida del tamaño total del mapa
    manzanas_crudas = bloque_solido.difference(red_de_calles)       # Sustracción booleana: recorte de la red de calles sobre la placa sólida

    if isinstance(manzanas_crudas, Polygon):                        # Evaluación de la tipología geométrica (detecta si quedó una sola manzana enorme)
        lista_manzanas = [manzanas_crudas]                          # Encapsulamiento del polígono solitario
    elif isinstance(manzanas_crudas, MultiPolygon):                 # Evaluación de la tipología (múltiples manzanas separadas por calles)
        lista_manzanas = list(manzanas_crudas.geoms)                # Extracción independiente de los fragmentos.
    else:
        lista_manzanas = []                                         # Filtro de seguridad ante corrupciones de geometría espacial.

    # --- ESCRITURA DEL SCRIPT PARA OCTAVE ---
    with open('Mapa_Obstaculos_RRT.m', 'w') as f:                   # Apertura del canal de escritura para exportación a formato Octave
        f.write("% Archivo generado vía Python (Espacio Negativo de Calles)\n") # Inserción de metadatos informativos
        f.write("obstaculos = {\n")                                 # Inicialización de la sintaxis estructurada para
        contador = 0                                                # Arranque del totalizador de polígonos exportados

        for manzana in lista_manzanas:                              # Análisis secuencial del listado de edificios obtenidos
            coords = list(manzana.exterior.coords)                  # Recuperación de los vértices delimitadores externos de la manzana
            if len(coords) > 3:                                     # Filtro de calidad geométrica (exclusión de puntos aislados o líneas sin área real)
                f.write("    [")                                    # Apertura del bloque de matriz para el polígono.
                v_strs = [f"{round(x, 2)}, {round(y, 2)}" for x, y in coords] # Redondeo de precisión a dos decimales y formateo textual de coordenadas
                f.write("; ".join(v_strs))                          # Ensamblaje de la cadena bajo los lineamientos sintácticos de matrices en
                f.write("];\n")                                     # Cierre formal del polígono actual
                contador += 1                                       # Incremento del conteo de bloques válidos procesados
        f.write("};\n")                                             # Clausura de la variable global de obstáculos

    print(f"Python: Se generaron {contador} manzanas como obstáculos.") # Confirmación estadística en consola

    # --- CREACIÓN DEL SISTEMA DE COLISIONES DISCRETO ---
    print("Python: Generando matriz de ocupación discreta (501x501)...") # Notificación de inicio de la fase de renderizado

    img = Image.new('L', (int(LADO_MAPA) + 1, int(LADO_MAPA) + 1), 0) # Creación de un lienzo digital oscuro (0 indica espacio libre transitable)
    draw = ImageDraw.Draw(img)                                          # Habilitación de la herramienta de dibujo en memoria gráfica

    for manzana in lista_manzanas:                                      # Recorrido iterativo sobre los polígonos estructurales confirmados
        coords = [(x, y) for x, y in manzana.exterior.coords]           # Extracción de vértices para orientación del render
        draw.polygon(coords, fill=1)                                    # Relleno cromático de la manzana (1 equivale a muro/colisión)

    matriz_ocupacion = np.array(img)                                    # Transformación de la imagen gráfica a una matriz numérica estricta
    np.savetxt('Mapa_Discreto.txt', matriz_ocupacion, fmt='%d')         # Volcado de la matriz binaria a un archivo de texto plano
    print("Python: Grid Caching guardado como 'Mapa_Discreto.txt'")   # Reporte de éxito en la escritura de caché

    # --- CREACIÓN DEL MAPA DE SEMILLAS (EJE MEDIAL) ---
    print("Python: Calculando el esqueleto de las calles...")           # Aviso de entrada a la fase de cálculo morfológico

    matriz_calles = (matriz_ocupacion == 0)                             # Inversión lógica condicional: aislamiento de las celdas marcadas como calle
    esqueleto = skeletonize(matriz_calles)                              # Aplicación de reducción morfológica para compactar la avenida a 1 pixel central de grosor

    matriz_esqueleto = np.uint8(esqueleto)                              # Moldeado de variables booleanas hacia formato numérico entero estándar (0 y 1)
    np.savetxt('Mapa_Esqueleto.txt', matriz_esqueleto, fmt='%d')        # Almacenamiento persistente del mapa base de navegación
    print("Python: Eje Medial guardado como 'Mapa_Esqueleto.txt'")    # Confirmación del último proceso generativo


if __name__ == "__main__":                                              # Verificación del contexto de ejecución (lanzamiento directo desde terminal vs importación pasiva)
    if len(sys.argv) > 1:                                               # Comprobación de captura del parámetro de ruta mediante los argumentos del sistema.
        procesar_mapa_calles(sys.argv[1])                               # Disparo de la función núcleo alimentada con la ruta proporcionada
    else:
        print("Error: No se proporcionó la ruta del archivo.")          # Emisión de alerta por deficiencia de argumentos de entrada
