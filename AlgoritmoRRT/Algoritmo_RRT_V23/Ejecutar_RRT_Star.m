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
x_limites = [0, 500];   % Define la anchura del espacio de trabajo, de 0 a 100 en el eje X.
y_limites = [0, 500];   % Define la altura del espacio de trabajo, de 0 a 100 en el eje Y.
inicio = [50, 50];      % Es el punto de partida del algoritmo.
objetivo = [450, 450];    % Es el punto al que el algoritmo debe llegar.
tolerancia = 15;         % Distancia máxima al objetivo para considerar que se ha alcanzado. (Ahora se usa como radio de conexión)
max_iter = 17000;        % Número máximo de intentos para encontrar un camino.
step_size = 12;        % La longitud de cada paso del algoritmo.
prob_objetivo = 0.10;    % Probabilidad de muestrear el punto objetivo (5%)
grid_size = 25;         % Tamaño de la celda de la cuadricula (10x10 unidades)
prob_bias = 0.12;        % Probabilidad del 12% de usar el muestreo por densidad
num_celdas_x = ceil((x_limites(2) - x_limites(1)) / grid_size);
num_celdas_y = ceil((y_limites(2) - y_limites(1)) / grid_size);


% =========================================================================
% === USAR ARCHIVO APARTE DONDE ESTAN DEFINIDOS LOS OBSTACULOS ===
% =========================================================================
run('Mapa_Obstaculos_RRT.m');


% =========================================================================
% === INICIALIZACIÓN DEL BOSQUE (RRT* MULTI-ÁRBOL) ===
% =========================================================================

num_arboles = 12; % 1 Inicio + 1 Objetivo + 10 Semillas

% 1. Generar 10 puntos seguros usando nuestra nueva función
disp('Generando semillas seguras para el bosque...');
semillas = Generar_Semillas_Libres(10, x_limites, y_limites, obstaculos);

% 2. Consolidar todos los puntos base (raíces de los árboles)
% Índice 1: Inicio | Índice 2: Objetivo | Índices 3 al 12: Semillas
raices_bosque = [inicio; objetivo; semillas];

% 3. Inicializar las estructuras de datos (Cell Arrays)
bosque_nodos = cell(num_arboles, 1);
bosque_padres = cell(num_arboles, 1);
bosque_mapas_densidad = cell(num_arboles, 1);

for k = 1:num_arboles
    bosque_nodos{k} = raices_bosque(k, :); % El primer nodo de cada árbol es su raíz
    bosque_padres{k} = 0;                  % La raíz no tiene padre
    bosque_mapas_densidad{k} = zeros(num_celdas_y, num_celdas_x);

    % Mapear densidad de la raíz inicial
    [c_idx, r_idx] = Mapeo_Densidad_Grid(raices_bosque(k, :), x_limites, y_limites, grid_size);
    bosque_mapas_densidad{k}(r_idx, c_idx) = 1;
end

% 4. Matriz de Adyacencia y Detalles de Conexión
% matriz_conexiones(i, j) será 1 si el árbol 'i' se conectó con el árbol 'j'
matriz_conexiones = zeros(num_arboles, num_arboles);

% detalles_conexiones{i, j} guardará [idx_en_arbol_i, idx_en_arbol_j] para saber por qué nodos se unieron
detalles_conexiones = cell(num_arboles, num_arboles);

% Variables de estado de la búsqueda
is_connected = false;
ruta_arboles = []; % Guardará la secuencia de árboles (ej. [1, 5, 8, 2])


% --- NUEVAS VARIABLES PARA EXPORTAR MÉTRICAS ---
nodos_colisionados = 0;     % Contador de intentos que chocaron con obstáculos.
iteraciones_totales = 0;    % Registra cuántas iteraciones se ejecutaron realmente.
% (Asegúrate de que is_connected esté declarada aquí también)

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

% --- Dibujar las semillas en el mapa ---
plot(semillas(:,1), semillas(:,2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 6);


% =========================================================================
%  === ALGORITMO RRT BOSQUE ===
% =========================================================================

% >>> INICIO DEL CONTEO DE TIEMPO PARA LA FASE DE BÚSQUEDA INICIAL <<<
tic;

for i = 1:max_iter
    iteraciones_totales = i;

    % --------------------------------------------------------------------------
    % --- 1. Lógica de Turnos (Round-Robin para 12 árboles) ---
    % --------------------------------------------------------------------------
    % mod(i-1, num_arboles) + 1 nos da una secuencia repetitiva: 1, 2, 3... 12, 1, 2...
    tree_id = mod(i-1, num_arboles) + 1;

    current_nodes = bosque_nodos{tree_id};
    current_parent = bosque_padres{tree_id};
    current_density_map = bosque_mapas_densidad{tree_id};

    % Para el muestreo, haremos que todos los árboles tengan un ligero bias
    % hacia el objetivo final (índice 2) para "jalarlos" hacia la meta.
    nodo_meta_bias = raices_bosque(2, :);

    % --------------------------------------------------------------------------
    % --- 2. Expansión del Árbol Actual ---
    % --------------------------------------------------------------------------
    rand_point = Generar_Punto_Aleatorio(nodo_meta_bias, current_density_map, x_limites, y_limites, grid_size, prob_objetivo, prob_bias);

    dif = current_nodes - rand_point;
    distancias = sqrt(sum(dif.^2, 2));
    [~, idx] = min(distancias);
    nearest_node = current_nodes(idx, :);

    direction = (rand_point - nearest_node);
    direction = direction / norm(direction);
    new_node = nearest_node + step_size * direction;

    if ~Verificar_Colision_Segmento(nearest_node, new_node, obstaculos, distancia_muestreo)

        % Añadir el nodo al árbol actual
        current_nodes = [current_nodes; new_node];
        current_parent(end+1) = idx;
        new_node_idx = size(current_nodes, 1);

        % Actualizar densidad
        [c_idx, r_idx] = Mapeo_Densidad_Grid(new_node, x_limites, y_limites, grid_size);
        current_density_map(r_idx, c_idx) = current_density_map(r_idx, c_idx) + 1;

        % Guardar cambios en el bosque
        bosque_nodos{tree_id} = current_nodes;
        bosque_padres{tree_id} = current_parent;
        bosque_mapas_densidad{tree_id} = current_density_map;

        % Dibujar la rama (Azul para inicio, Cyan para objetivo, Verde para semillas)
        if tree_id == 1
            color_rama = 'b';
        elseif tree_id == 2
            color_rama = 'c';
        else
            color_rama = 'g';
        end
        plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], color_rama);
        drawnow;


        % --------------------------------------------------------------------------
        % --- 3. INTENTO DE CONEXIÓN CON EL RESTO DEL BOSQUE ---
        % --------------------------------------------------------------------------
        for otro_id = 1:num_arboles
            % No conectarse a sí mismo, ni a árboles con los que YA está conectado
            if otro_id == tree_id || matriz_conexiones(tree_id, otro_id) == 1
                continue;
            end

            other_nodes = bosque_nodos{otro_id};
            dif_other = other_nodes - new_node;
            distancias_other = sqrt(sum(dif_other.^2, 2));
            [min_dist, idx_nearest_other] = min(distancias_other);

            % Si está lo suficientemente cerca...
            if min_dist < tolerancia
                % Y si no hay pared entre ellos...
                if ~Verificar_Colision_Segmento(new_node, other_nodes(idx_nearest_other, :), obstaculos, distancia_muestreo)

                    % ¡SE DIERON LA MANO! (Registramos la conexión)
                    matriz_conexiones(tree_id, otro_id) = 1;
                    matriz_conexiones(otro_id, tree_id) = 1;

                    % Guardamos EXACTAMENTE qué nodos se unieron
                    detalles_conexiones{tree_id, otro_id} = [new_node_idx, idx_nearest_other];
                    detalles_conexiones{otro_id, tree_id} = [idx_nearest_other, new_node_idx];

                    plot([new_node(1), other_nodes(idx_nearest_other, 1)], [new_node(2), other_nodes(idx_nearest_other, 2)], 'm--', 'LineWidth', 1.5);
                    drawnow;
                    disp(['> Conexión formada entre el árbol ', num2str(tree_id), ' y el árbol ', num2str(otro_id)]);

                    % ------------------------------------------------------------------
                    % --- 4. VERIFICAR SI LA RED YA CONECTÓ EL INICIO (1) Y OBJETIVO (2)
                    % ------------------------------------------------------------------
                    ruta_arboles = BFS_Encontrar_Ruta(matriz_conexiones, 1, 2);

                    if ~isempty(ruta_arboles)
                        is_connected = true; % Usamos tu misma variable original
                        disp('¡RED COMPLETADA! Existe un camino continuo desde el Inicio hasta el Objetivo.');
                        disp(['Secuencia de árboles conectados: ', num2str(ruta_arboles)]);
                        break; % Rompe el for de conexiones
                    end
                end
            end
        end % Fin for conexiones

        if is_connected
            break; % Rompe el bucle principal for max_iter
        end
    else
        nodos_colisionados = nodos_colisionados + 1;
    end
end

% >>> FIN DEL CONTEO DE TIEMPO PARA LA BÚSQUEDA INICIAL <<<
tiempo_busqueda_inicial = toc;


if is_connected
    disp('Unificando bosque y extrayendo ruta inicial...');

    % ----------------------------------------------------------------------
    % === 1. UNIFICAR BOSQUE Y RECONSTRUIR CAMINO INICIAL ===
    % ----------------------------------------------------------------------
    [nodes_global, parent_global, path_coords_inicial, path_indices_global] = Unificar_Bosque_RRT(bosque_nodos, bosque_padres, detalles_conexiones, num_arboles);

    total_nodos_encontrados = size(nodes_global, 1);

    % ----------------------------------------------------------------------
    % === 2. DIBUJAR CAMINO INICIAL COMPLETO ===
    % ----------------------------------------------------------------------
    Limpiar_Dibujos_RRT('union');

    % Dibujamos la ruta inicial en rojo continuo (ancho 2)
    plot(path_coords_inicial(:,1), path_coords_inicial(:,2), 'r-', 'LineWidth', 2);
    plot(path_coords_inicial(:,1), path_coords_inicial(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');
    drawnow;

    % --- CALCULAR COSTO DE LA RUTA INICIAL ---
    costo_inicial = 0;
    for k = 1:size(path_coords_inicial, 1) - 1
        costo_inicial = costo_inicial + norm(path_coords_inicial(k+1, :) - path_coords_inicial(k, :));
    end

    % ----------------------------------------------------------------------
    % === FASE DE OPTIMIZACIÓN RRT* (POST-PROCESAMIENTO SHORTCUTTING) ===
    % ----------------------------------------------------------------------
    disp('Iniciando optimización de caminos (Suavizado por atajos)...');
    tic;

    % Le pasamos directamente las coordenadas iniciales, sin complicaciones
    path_final_coords_completo = Optimizar_Ruta_Final(path_coords_inicial, obstaculos, @Verificar_Colision_Segmento, distancia_muestreo);

    tiempo_optimizacion = toc;

    % ----------------------------------------------------------------------
    % === DIBUJAR EL CAMINO FINAL OPTIMIZADO ===
    % ----------------------------------------------------------------------
    Limpiar_Dibujos_RRT('fase_optimizacion');

    % Dibujar ruta final óptima (línea roja gruesa)
    plot(path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'r-', 'LineWidth', 2.5);
    plot(path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');

    % IMPORTANTE: Esto fuerza a MATLAB a dibujar la línea roja inmediatamente
    drawnow;

    nodos_camino_final = size(path_final_coords_completo, 1);
    disp('Optimización finalizada. Camino más corto trazado.');

    % ==================================================================
    % === EXPORTACIÓN A CARPETA DE RESULTADOS ===
    % ==================================================================
    Exportar_Reporte_Metricas(path_final_coords_completo, tiempo_busqueda_inicial, tiempo_optimizacion, total_nodos_encontrados, nodos_camino_final, iteraciones_totales, nodos_colisionados, costo_inicial);
else
    disp('No se encontró conexión dentro del número máximo de iteraciones.');
end
