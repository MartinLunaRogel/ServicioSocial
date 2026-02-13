===========================================================================
   PROYECTO: ALGORITMO RRT*
===========================================================================
Autor: Martin Luna Rogel
Objetivo: Planificación de rutas autónomas en entornos con obstáculos.
===========================================================================

---------------------------------------------------------------------------
1. DESCRIPCIÓN GENERAL DEL PROYECTO
---------------------------------------------------------------------------
Este proyecto consiste en el desarrollo y optimización de un sistema de 
planificación de rutas autónomas para entornos complejos utilizando una 
arquitectura de software modular en GNU Octave. 

El núcleo del sistema es un algoritmo RRT Bidireccional (Rapidly-exploring 
Random Tree), el cual soluciona el problema de navegación mediante la 
expansión simultánea de dos árboles de búsqueda: uno que nace en el punto 
de inicio y otro en el punto objetivo. Esta técnica reduce 
significativamente el tiempo de búsqueda en comparación con el RRT estándar 
al buscar un "nodo de unión" en el espacio libre.



El proyecto integra varios pilares de innovación avanzada:

A) INTELIGENCIA DE EXPLORACIÓN:
   Se implementó un sistema de "Bias" (sesgo) doble. El primero orienta el 
   muestreo hacia el objetivo para acelerar la convergencia. El 
   segundo utiliza un Mapa de Densidad basado en rejilla (Grid) que detecta 
   zonas poco exploradas, obligando al algoritmo a salir de "trampas" y 
   cubrir todo el mapa de forma eficiente.

B) OPTIMIZACIÓN DE TRAYECTORIA (RRT*):
   Una vez establecida la conexión inicial, el sistema no se detiene. Se 
   aplica un post-procesamiento de "Rewiring" o reconexión. Este 
   módulo analiza cada nodo del camino encontrado e intenta conectarlo a 
   mejores padres dentro del árbol para reducir la distancia euclidiana 
   total, transformando una ruta errática en una trayectoria eficiente y 
   suave.

C) ROBUSTEZ GEOMÉTRICA:
   La seguridad del robot se gestiona mediante un motor de colisiones que 
   evolucionó de un muestreo discreto de puntos a una verificación de 
   intersección de aristas matemáticas. Esto garantiza que el 
   camino final sea 100% seguro, incluso al pasar por pasajes estrechos o 
   esquinas afiladas de obstáculos poligonales.


D) ANÁLISIS CIENTÍFICO:
   El sistema incluye un generador automático de métricas que registra el 
   desempeño computacional (tiempos de CPU), la calidad de la ruta (costo) 
   y la eficiencia del muestreo, permitiendo una evaluación objetiva de 
   cada simulación.

---------------------------------------------------------------------------
2. ARQUITECTURA MODULAR DEL SISTEMA (12 ARCHIVOS)
---------------------------------------------------------------------------
El proyecto ha sido completamente modularizado para garantizar que sea 
fácilmente entendible y mantenible por futuros desarrolladores:

[ NÚCLEO DEL ALGORITMO ]
* Ejecutar_RRT_Star.m      : Script maestro que coordina la expansión de
                             los dos árboles y la lógica principal.
* Verificar_Colision_Segmento.m : Detecta colisiones verificando si la ruta
                             intersecta con las paredes de los polígonos.
* Optimizar_Ruta_Final.m   : Proceso de 'Rewiring' (RRT*) que busca reducir
                             el costo total de la ruta encontrada.

[ INTELIGENCIA Y DATOS ]
* Generar_Punto_Aleatorio.m: Gestiona el muestreo usando Bias de Objetivo
                             y Bias de Densidad de nodos.
* Mapeo_Densidad_Grid.m    : Convierte coordenadas en una cuadrícula para
                             medir la saturación del espacio.
* Calcular_Costo_De_Ruta.m : Calcula la distancia euclidiana acumulada 
                             desde cualquier nodo hasta su raíz.

[ GESTIÓN GRÁFICA Y RESULTADOS ]
* Inicializar_Graficos_RRT.m : Configura la ventana, los límites y dibuja
                               el mapa de obstáculos inicial.
* Dibujar_Ruta_Final.m     : Concatena las ramas de los árboles y traza la
                             trayectoria definitiva en pantalla.
* Limpiar_Dibujos_RRT.m    : Gestiona el refresco visual borrando líneas
                             temporales durante la optimización.
* Exportar_Reporte_Metricas.m : Genera informes automáticos (.txt) en la
                                carpeta '../Resultados_Metricas/'.

[ CONFIGURACIÓN Y UTILIDADES ]
* Mapa_Obstaculos_RRT.m    : Definición de los polígonos del entorno 
                             (Patrón de damero 8x8).
* Reconstruir_Camino_RRT.m : Función que rastrea la cadena de padres desde
                             la unión hasta el punto de origen.


---------------------------------------------------------------------------
3. INSTRUCCIONES DE EJECUCIÓN
---------------------------------------------------------------------------
1. Iniciar GNU Octave y cargar los paquetes.
2. Ejecutar el script: Ejecutar_RRT_Star.m.
3. Al finalizar, revisar la carpeta 'Resultados_Metricas' para consultar el 
   desempeño detallado de la simulación (tiempos, costos y eficiencia).

---------------------------------------------------------------------------
                           UAQ - SERVICIO SOCIAL
---------------------------------------------------------------------------