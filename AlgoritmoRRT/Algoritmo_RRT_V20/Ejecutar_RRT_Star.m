% Algoritmo RRT con un primer obstaculos
% Martin Luna Rogel%
%
% Descripción:
%   Algoritmo RRT Bidireccional (Rapidly-exploring Random Tree).
%   El algoritmo genera dos árboles (uno desde inicio, otro desde objetivo) que crecen
%   simultáneamente hasta encontrarse en un "nodo de unión", buscando un camino óptimo.
%   Se añadioeron varios obstaculos cuadrados por todo el area, para que trate de rodearlos y encontrar el punto final
%
% Entradas:
%   - Ninguna.
%
% Salidas:
%   - Visualización grafica:
%       • Punto inicial (verde).
%       • Punto final (rojo).
%       • Expansión del árbol (líneas azules).
%       • Camino final encontrado (línea roja ).
%       • Obstaculo (barra al cento de la grafica)
clc; clear; close all;
pkg load statistics;
pkg load geometry;

% =========================================================================
%  === PARAMETROS DEL ESPACIO Y DEL ALGORITMO ===
% =========================================================================
x_limites = [0, 100];   % Define la anchura del espacio de trabajo, de 0 a 100 en el eje X.
y_limites = [0, 100];   % Define la altura del espacio de trabajo, de 0 a 100 en el eje Y.
inicio = [10, 20];      % Es el punto de partida del algoritmo.
objetivo = [90, 80];    % Es el punto al que el algoritmo debe llegar.
tolerancia = 3;         % Distancia máxima al objetivo para considerar que se ha alcanzado. (Ahora se usa como radio de conexión)
max_iter = 7000;        % Número máximo de intentos para encontrar un camino.
step_size = 3;        % La longitud de cada paso del algoritmo.
prob_objetivo = 0.10;    % Probabilidad de muestrear el punto objetivo (5%)
grid_size = 5;         % Tamaño de la celda de la cuadricula (10x10 unidades)
prob_bias = 0.12;        % Probabilidad del 12% de usar el muestreo por densidad
num_celdas_x = ceil((x_limites(2) - x_limites(1)) / grid_size);
num_celdas_y = ceil((y_limites(2) - y_limites(1)) / grid_size);

% =========================================================================
%  === INICIALIZAR ESTRUCTURAS DOBLES (RRT BIDIRECCIONAL) ===
% =========================================================================

% --- ESTRUCTURAS DEL ÁRBOL DE INICIO (T_inicio) ---
nodes_inicio = inicio;      % Lista de nodos del árbol que parte del inicio.
parent_inicio = 0;          % Lista de padres para T_inicio.
parent_inicio(1) = 0;       % El primer punto no tiene padre.
node_density_map_inicio = zeros(num_celdas_y, num_celdas_x); % Mapa de densidad para T_inicio.

% --- ESTRUCTURAS DEL ÁRBOL OBJETIVO (T_objetivo) ---
nodes_objetivo = objetivo;  % Lista de nodos del árbol que parte del objetivo.
parent_objetivo = 0;        % Lista de padres para T_objetivo.
parent_objetivo(1) = 0;     % El primer punto (objetivo) no tiene padre.
node_density_map_objetivo = zeros(num_celdas_y, num_celdas_x); % Mapa de densidad para T_objetivo.

% Inicializa un arreglo para guardar los valores aleatorios del random (se usará el de inicio)
valores_x_aleatorios = [];  % Valores de las coordenadas x
valores_y_aleatorios = [];  % Valores de las coordenadas y

% Variables de conexión
is_connected = false;       % Bandera para saber si se ha encontrado el camino.
idx_union_inicio = 0;       % Índice del nodo de unión en T_inicio.
idx_union_objetivo = 0;     % Índice del nodo de unión en T_objetivo (el más cercano).

% --- NUEVAS VARIABLES PARA EXPORTAR MÉTRICAS ---
nodos_colisionados = 0;     % Contador de intentos que chocaron con obstáculos.
iteraciones_totales = 0;    % Registra cuántas iteraciones se ejecutaron realmente.
% (Asegúrate de que is_connected esté declarada aquí también)

% =========================================================================
% === USAR ARCHIVO APARTE DONDE ESTAN DEFINIDOS LOS OBSTACULOS ===
% =========================================================================

run('Mapa_Obstaculos_RRT.m');


% =========================================================================
% === AJUSTE DE LA DISTANCIA DE MUESTREO PARA POLÍGONOS ===
% =========================================================================
% Se establece un valor fijo bajo para garantizar la detección de colisiones
% en pasajes estrechos con obstáculos poligonales.
distancia_muestreo = 1.0; % Define una distancia de muestreo baja (0.5 unidades) para la verificación de colisión


% =========================================================================
%  === DIBUJAR EN EL ESPACIO DE TRABAJO (MODULARIZADO) ===
% =========================================================================

Inicializar_Graficos_RRT(x_limites, y_limites, inicio, objetivo, obstaculos);


% =========================================================================
%  === INICIALIZACIÓN DEL MAPA DE DENSIDAD ===
% =========================================================================

% Contar el nodo de inicio en el mapa
[col_idx_inicio, row_idx_inicio] = Mapeo_Densidad_Grid(inicio, x_limites, y_limites, grid_size);
node_density_map_inicio(row_idx_inicio, col_idx_inicio) = node_density_map_inicio(row_idx_inicio, col_idx_inicio) + 1;

% Contar el nodo objetivo en su mapa de densidad (T_objetivo)
[col_idx_objetivo, row_idx_objetivo] = Mapeo_Densidad_Grid(objetivo, x_limites, y_limites, grid_size);
node_density_map_objetivo(row_idx_objetivo, col_idx_objetivo) = node_density_map_objetivo(row_idx_objetivo, col_idx_objetivo) + 1;


% =========================================================================
%  === ALGORITMO RRT BIDIRECCIONAL ===
% =========================================================================

% >>> INICIO DEL CONTEO DE TIEMPO PARA LA FASE DE BÚSQUEDA INICIAL <<<
tic;

% El bucle principal donde se ejecuta la lógica de expansión del árbol.
for i = 1:max_iter % El bucle se repite hasta el maximo de iteraciones definido.

    iteraciones_totales = i; % Registra la iteración actual en cada ciclo.

    % --------------------------------------------------------------------------
    % --- Lógica de Alternancia: Se define qué árbol crece en esta iteración ---
    if mod(i, 2) == 1 % Iteración impar: crece T_inicio (del inicio al objetivo)
        current_nodes = nodes_inicio;
        current_parent = parent_inicio;
        current_density_map = node_density_map_inicio;
        current_tree_color = 'b'; % Color para T_inicio
        other_nodes = nodes_objetivo; % El árbol al que intenta conectarse es T_objetivo
        other_parent = parent_objetivo;
        tree_id = 1; % 1 para T_inicio
    else % Iteración par: crece T_objetivo (del objetivo al inicio)
        current_nodes = nodes_objetivo;
        current_parent = parent_objetivo;
        current_density_map = node_density_map_objetivo;
        current_tree_color = 'c'; % Color para T_objetivo (cyan)
        other_nodes = nodes_inicio; % El árbol al que intenta conectarse es T_inicio
        other_parent = parent_inicio;
        tree_id = 2; % 2 para T_objetivo
    end
    % --------------------------------------------------------------------------

    % --------------------------------------------------------------------------
    % Generar un punto aleatorio en el espacio (Muestreo con Bias - MODULARIZADO)
    rand_point = Generar_Punto_Aleatorio(other_nodes, current_density_map, x_limites, y_limites, grid_size, prob_objetivo, prob_bias);

    % Guarda las coordenadas X e Y en los arreglos (Solo para T_inicio, si fuera necesario)
    if tree_id == 1
        valores_x_aleatorios = [valores_x_aleatorios; rand_point(1)];
        valores_y_aleatorios = [valores_y_aleatorios; rand_point(2)];
    end

    % --------------------------------------------------------------------------
    % Buscar el nodo más cercano en el árbol actual
    dif = current_nodes - rand_point;                  % Calcula la distancia entre el punto aleatorio y CADA uno de los nodos.
    distancias = sqrt(sum(dif.^2, 2));         % Calcula la distancia euclidiana.
    [~, idx] = min(distancias);                % Encuentra la posición (`idx`) del nodo con la distancia más corta.
    nearest_node = current_nodes(idx, :);              % Coordenadas del nodo más cercano en el árbol actual.

    % --------------------------------------------------------------------------
    % Avanzar desde el nodo más cercano hacia el punto aleatorio
    direction = (rand_point - nearest_node);
    direction = direction / norm(direction);           % Normaliza el vector.
    new_node = nearest_node + step_size * direction;   % Calcula las coordenadas del nuevo punto a `step_size`.

    % --------------------------------------------------------------------------
    % Verificar colision con el obstaculo
    if ~Verificar_Colision_Segmento(nearest_node, new_node, obstaculos, distancia_muestreo)

        % Si no hay colisión, el nuevo nodo se añade al árbol actual.
        current_nodes = [current_nodes; new_node];      % Agrega el nuevo nodo a la lista de todos los nodos.
        current_parent(end+1) = idx;            % Registra quién es el "padre" de este nuevo nodo.
        new_node_idx_in_current_tree = size(current_nodes, 1); % Guarda el índice del nuevo nodo.

        % Actualizar el Mapa de Densidad con el nuevo nodo
        [col_idx_new_node, row_idx_new_node] = Mapeo_Densidad_Grid(new_node, x_limites, y_limites, grid_size);
        current_density_map(row_idx_new_node, col_idx_new_node) = current_density_map(row_idx_new_node, col_idx_new_node) + 1;

        % Dibujar la conexión (en el color del árbol actual: 'b' o 'c')
        plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], current_tree_color);
        drawnow;        % Actualiza el gráfico en tiempo real.


        % --------------------------------------------------------------------------
        % === COMPROBACIÓN DE CONEXIÓN AL OTRO ÁRBOL (Nuevo paso Bidireccional) ===
        % --------------------------------------------------------------------------

        % 1. Buscar el nodo más cercano en el OTRO árbol
        dif_other = other_nodes - new_node;
        distancias_other = sqrt(sum(dif_other.^2, 2));
        [min_dist_to_other, idx_nearest_in_other] = min(distancias_other);
        nearest_other_node = other_nodes(idx_nearest_in_other, :);

        % 2. Verificar si la distancia es menor que la tolerancia (o step_size) Y si el segmento de unión es libre de colisión
        if min_dist_to_other < tolerancia % Usamos tolerancia como radio de conexión
            if ~Verificar_Colision_Segmento(new_node, nearest_other_node, obstaculos, distancia_muestreo)

                disp('Objetivo alcanzado! Conexión entre árboles encontrada.');
                is_connected = true;

                % Registra los índices del 'nodo de unión'
                if tree_id == 1 % T_inicio creció y se conectó a T_objetivo
                    idx_union_inicio = new_node_idx_in_current_tree; % Índice del new_node en T_inicio
                    idx_union_objetivo = idx_nearest_in_other;       % Índice del nodo más cercano en T_objetivo
                else % T_objetivo creció y se conectó a T_inicio
                    idx_union_objetivo = new_node_idx_in_current_tree; % Índice del new_node en T_objetivo
                    idx_union_inicio = idx_nearest_in_other;           % Índice del nodo más cercano en T_inicio
                end

                % Dibuja el segmento de unión (en un color que resalte, e.g., magenta)
                plot([new_node(1), nearest_other_node(1)], [new_node(2), nearest_other_node(2)], 'm--', 'LineWidth', 1.5);
                drawnow;

            end
        end

        % --------------------------------------------------------------------------
        % --- Almacenar los cambios de vuelta en las estructuras principales ---
        if tree_id == 1
            nodes_inicio = current_nodes;
            parent_inicio = current_parent;
            node_density_map_inicio = current_density_map;
        else
            nodes_objetivo = current_nodes;
            parent_objetivo = current_parent;
            node_density_map_objetivo = current_density_map;
        end
        % --------------------------------------------------------------------------

        % Si se conectó, salir del bucle
        if is_connected
            break;
        end
    else
      % === NUEVO: REGISTRO DE COLISIÓN ===
       nodos_colisionados = nodos_colisionados + 1; % Suma 1 cada vez que el paso es bloqueado.
    end
end

% >>> FIN DEL CONTEO DE TIEMPO PARA LA BÚSQUEDA INICIAL <<<
tiempo_busqueda_inicial = toc;


if is_connected
    % Capturar el número total de nodos en los árboles
    total_nodos_encontrados = size(nodes_inicio, 1) + size(nodes_objetivo, 1);

    % ----------------------------------------------------------------------
    % === 1. RECONSTRUIR EL CAMINO INICIAL (MODULARIZADO) ===
    % ----------------------------------------------------------------------

    % Obtener camino para el árbol de inicio
    [path_coords_inicio, path_indices_inicio] = Reconstruir_Camino_RRT(idx_union_inicio, nodes_inicio, parent_inicio);

    % Obtener camino para el árbol objetivo
    [path_coords_objetivo, path_indices_objetivo] = Reconstruir_Camino_RRT(idx_union_objetivo, nodes_objetivo, parent_objetivo);


    % ----------------------------------------------------------------------
    % === 2. DIBUJAR CAMINO INICIAL COMPLETO (MODULARIZADO) ===
    % ----------------------------------------------------------------------

    Limpiar_Dibujos_RRT('union');

    path_coords_inicial_completo = Dibujar_Ruta_Final(path_coords_inicio, path_coords_objetivo, 'inicial');


    % ----------------------------------------------------------------------
    % === FASE DE OPTIMIZACIÓN RRT* (POST-PROCESAMIENTO DOBLE) ===
    % ----------------------------------------------------------------------
    disp('Iniciando optimización de caminos (RRT* post-procesamiento)...');

    % >>> INICIO DEL CONTEO DE TIEMPO PARA LA OPTIMIZACIÓN <<<
    tic;

    % --- OPTIMIZACIÓN 1: Camino de INICIO a UNIÓN ---
    disp('Optimizando T_inicio...');
    new_parent_inicio = Optimizar_Ruta_Final(nodes_inicio, parent_inicio, path_indices_inicio, obstaculos, @Verificar_Colision_Segmento, distancia_muestreo);

    % --- OPTIMIZACIÓN 2: Camino de OBJETIVO a UNIÓN ---
    disp('Optimizando T_objetivo...');
    new_parent_objetivo = Optimizar_Ruta_Final(nodes_objetivo, parent_objetivo, path_indices_objetivo, obstaculos, @Verificar_Colision_Segmento, distancia_muestreo);

    % >>> FIN DEL CONTEO DE TIEMPO PARA LA OPTIMIZACIÓN <<<
    tiempo_optimizacion = toc;

    % ----------------------------------------------------------------------
    % === RECONSTRUIR Y DIBUJAR EL CAMINO FINAL OPTIMIZADO ===
    % ----------------------------------------------------------------------

    Limpiar_Dibujos_RRT('fase_optimizacion');


    % --- RECONSTRUIR CAMINO OPTIMIZADO (MODULARIZADO) ---
    [path_final_coords_inicio, ~] = Reconstruir_Camino_RRT(idx_union_inicio, nodes_inicio, new_parent_inicio);
    [path_final_coords_objetivo, ~] = Reconstruir_Camino_RRT(idx_union_objetivo, nodes_objetivo, new_parent_objetivo);


    % --- DIBUJAR RUTA FINAL ÓPTIMA (MODULARIZADO) ---
    path_final_coords_completo = Dibujar_Ruta_Final(path_final_coords_inicio, path_final_coords_objetivo, 'final');

    nodos_camino_final = size(path_final_coords_completo, 1);

    disp('Optimización finalizada. Camino más corto trazado.');

    % ==================================================================
    % === EXPORTACIÓN A CARPETA DE RESULTADOS (MODULARIZADO) ===
    % ==================================================================

    Exportar_Reporte_Metricas(path_final_coords_completo, tiempo_busqueda_inicial, tiempo_optimizacion, total_nodos_encontrados, nodos_camino_final, iteraciones_totales, nodos_colisionados);

else
    disp('No se encontró conexión dentro del número máximo de iteraciones.');
end
