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
prob_objetivo = 0.05;    % Probabilidad de muestrear el punto objetivo (5%)
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


% =========================================================================
% === USAR ARCHIVO APARTE DONDE ESTAN DEFINIDOS LOS OBSTACULOS ===
% =========================================================================

run('Mapa_Obstaculos_RRT.m');


% =========================================================================
% === AJUSTE DE LA DISTANCIA DE MUESTREO PARA POLÍGONOS ===
% =========================================================================
% Se establece un valor fijo bajo para garantizar la detección de colisiones
% en pasajes estrechos con obstáculos poligonales.
distancia_muestreo = 0.5; % Define una distancia de muestreo baja (0.5 unidades) para la verificación de colisión


% =========================================================================
%  === DIBUJAR EN ESL ESPACIO DE TRABAJO ===
% =========================================================================
figure;         % Abre una nueva ventana de gráfico.
hold on;        % Permite dibujar múltiples elementos en la misma ventana, como el punto de inicio, el obstáculo y las líneas del árbol.
grid on;        % Muestra una cuadrícula.
xlim(x_limites);    % Fija el límite horizontal del gráfico.
ylim(y_limites);    % Fija el límite vertical del gráfico.
plot(inicio(1), inicio(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');         % Dibuja el punto de inicio en el gráfico como un círculo verde.
plot(objetivo(1), objetivo(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');     % Dibuja el punto de llegada como un círculo rojo.
title('Algoritmo RRT Bidireccional con Bias de Densidad y RRT*');   % Añade un título en la parte superior del gráfico.
xlabel('X');    % Etiqueta el eje X.
ylabel('Y');    % Etiqueta el eje Y.

% Dibujar los obstaculos
% Se itera sobre el cell array 'obstaculos' para dibujar cada figura
for i = 1:size(obstaculos, 1)

    % Extraer los vértices del obstáculo actual (obs_i)
    vertices = obstaculos{i, 1};    % Esta línea parece correcta

    % Dibujar el obstáculo usando los límites extraídos
    patch(vertices(:,1), vertices(:,2), 'k', 'FaceAlpha', 0.3, 'EdgeColor', 'k'); % Se añade el punto y coma final para cerrar la instrucción
end


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
    % Generar un punto aleatorio en el espacio (Muestreo con Bias)
    r_prob = rand;

    if r_prob < prob_objetivo     % 1. Muestrear el punto objetivo (usamos el punto de origen del OTRO árbol)
        rand_point = other_nodes(1, :);

    elseif r_prob < prob_objetivo + prob_bias   % 2. Muestreo con el bias de exploracion (usando el mapa de densidad del árbol actual)

        % Crear una matriz de probabilidades inversas (favorables a baja densidad)
        epsilon = 0.01;       % Evita division por cero y asegura que las celdas con 1 nodo tenga mas probabilidad > 0
        prob_matrix = 1 ./ (current_density_map.^2 + epsilon);

        % Normalizar la matriz de probabilidades
        prob_vector = prob_matrix(:); % Convierte la matriz en un vector
        prob_vector = prob_vector / sum(prob_vector);

        % Seleccionar una celda de la cuadrícula basándose en estas probabilidades
        selected_cell_idx = randsample(numel(prob_vector), 1, true, prob_vector);

        % Convertir el índice lineal (de randsample) a índices de fila y columna
        [row_idx, col_idx] = ind2sub(size(current_density_map), selected_cell_idx);

        % Generar el punto aleatorio UNIFORMEMENTE dentro de la celda seleccionada
        x_min_cell = (col_idx - 1) * grid_size + x_limites(1);
        y_min_cell = (row_idx - 1) * grid_size + y_limites(1);

        rand_x = x_min_cell + rand * grid_size;
        rand_y = y_min_cell + rand * grid_size;

        % Asegurar que el punto no exceda los límites globales
        rand_x = min(rand_x, x_limites(2));
        rand_y = min(rand_y, y_limites(2));

        rand_point = [rand_x, rand_y];

    else % 3. Muestreo uniforme estándar (el resto del tiempo)
        rand_point = [rand*(x_limites(2)-x_limites(1)), ...
                      rand*(y_limites(2)-y_limites(1))];
    end

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
    end
end

% >>> FIN DEL CONTEO DE TIEMPO PARA LA BÚSQUEDA INICIAL <<<
tiempo_busqueda_inicial = toc;


if is_connected
    % Capturar el número total de nodos en los árboles
    total_nodos_encontrados = size(nodes_inicio, 1) + size(nodes_objetivo, 1);

    % ----------------------------------------------------------------------
    % === 1. RECONSTRUIR EL CAMINO INICIAL (ÍNDICES) ===
    % ----------------------------------------------------------------------

    % --- Camino de INICIO a UNIÓN (T_inicio) ---
    path_indices_inicio = [];
    p_inicio = idx_union_inicio; % El nodo de unión es el final de T_inicio

    while parent_inicio(p_inicio) ~= 0
        path_indices_inicio = [p_inicio; path_indices_inicio]; % Guarda el índice del nodo
        p_inicio = parent_inicio(p_inicio);
    end
    path_indices_inicio = [1; path_indices_inicio]; % Añadir el nodo de inicio (índice 1) al principio
    path_coords_inicio = nodes_inicio(path_indices_inicio, :);

    % --- Camino de OBJETIVO a UNIÓN (T_objetivo) ---
    path_indices_objetivo = [];
    p_objetivo = idx_union_objetivo; % El nodo de unión es el final de T_objetivo

    while parent_objetivo(p_objetivo) ~= 0
        path_indices_objetivo = [p_objetivo; path_indices_objetivo]; % Guarda el índice del nodo
        p_objetivo = parent_objetivo(p_objetivo);
    end
    path_indices_objetivo = [1; path_indices_objetivo]; % Añadir el nodo objetivo (índice 1 de T_objetivo) al principio
    path_coords_objetivo = nodes_objetivo(path_indices_objetivo, :);

    % --- Camino FINAL Completo (Concatenado) ---
    % Se invierte el camino del objetivo y se quita el nodo de unión duplicado (que es el primero)
    % Se remueve el primer nodo (objetivo) y el último nodo (el de unión) para evitar duplicados.
    path_coords_objetivo_invertido = flipud(path_coords_objetivo(2:end, :));
    path_coords_inicial_completo = [path_coords_inicio; path_coords_objetivo_invertido];


    % ----------------------------------------------------------------------
    % === 3. DIBUJAR CAMINO INICIAL COMPLETO (SIN OPTIMIZAR) ===
    % ----------------------------------------------------------------------
    % Borrar la línea de unión magenta para reemplazarla con la línea roja.
    h_union_line = findobj(gca, 'Type', 'line', 'Color', 'm', 'LineWidth', 1.5);
    delete(h_union_line);

    plot(path_coords_inicial_completo(:,1), path_coords_inicial_completo(:,2), 'r-', 'LineWidth', 2); % Dibuja la ruta inicial completa en rojo
    plot(path_coords_inicial_completo(:,1), path_coords_inicial_completo(:,2), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'black'); % Remarcar los nodos


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

    % **PASO CRÍTICO: BORRAR DIBUJOS PREVIOS DEL CAMINO**
    % 1. Borrar la línea de trayectoria del camino inicial (LineWidth=2)
    h_lines = findobj(gca, 'Type', 'line', 'LineWidth', 2);
    delete(h_lines);
    % También borramos las líneas temporales verdes de la optimización (g-- con LineWidth=1.5)
    h_temp_lines = findobj(gca, 'Type', 'line', 'Color', 'g', 'LineWidth', 1.5);
    delete(h_temp_lines);

    % 2. Borrar los marcadores de los nodos del camino inicial (MarkerSize=4)
    h_markers = findobj(gca, 'Type', 'line', 'MarkerSize', 4);
    delete(h_markers);


    % --- RECONSTRUIR CAMINO OPTIMIZADO (INICIO a UNIÓN) ---
    final_path_indices_inicio = [];
    p_final_inicio = idx_union_inicio;

    while new_parent_inicio(p_final_inicio) ~= 0
        final_path_indices_inicio = [p_final_inicio; final_path_indices_inicio];
        p_final_inicio = new_parent_inicio(p_final_inicio);
    end
    final_path_indices_inicio = [1; final_path_indices_inicio]; % Asegurar que el nodo de inicio está al principio
    path_final_coords_inicio = nodes_inicio(final_path_indices_inicio, :);


    % --- RECONSTRUIR CAMINO OPTIMIZADO (OBJETIVO a UNIÓN) ---
    final_path_indices_objetivo = [];
    p_final_objetivo = idx_union_objetivo;

    while new_parent_objetivo(p_final_objetivo) ~= 0
        final_path_indices_objetivo = [p_final_objetivo; final_path_indices_objetivo];
        p_final_objetivo = new_parent_objetivo(p_final_objetivo);
    end
    final_path_indices_objetivo = [1; final_path_indices_objetivo]; % Asegurar que el nodo objetivo (T_objetivo indice 1) está al principio
    path_final_coords_objetivo = nodes_objetivo(final_path_indices_objetivo, :);

    % --- CONCATENAR EL CAMINO FINAL ÓPTIMO ---
    % Se invierte la rama objetivo y se concatena, eliminando el nodo de unión duplicado.
    % La rama de inicio ya contiene el nodo de unión.
    path_final_coords_objetivo_invertido = flipud(path_final_coords_objetivo(2:end, :));
    path_final_coords_completo = [path_final_coords_inicio; path_final_coords_objetivo_invertido];
    nodos_camino_final = size(path_final_coords_completo, 1);


    % 2. Dibujar la ruta final ÓPTIMA en rojo y más gruesa (la trayectoria)
    plot(path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'r-', 'LineWidth', 2);

    % 3. Remarcar TODOS los nodos del camino final (los puntos individuales)
    plot(path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');

    disp('Optimización finalizada. Camino más corto trazado.');

    % ==================================================================
    % === MOSTRAR MÉTRICAS DE RENDIMIENTO REQUERIDAS ===
    % ==================================================================
    fprintf('\n\n--- MÉTRICAS DE RENDIMIENTO ---\n');
    fprintf('1. Tiempo de búsqueda inicial (RRT Bidireccional): %.4f segundos\n', tiempo_busqueda_inicial);
    fprintf('2. Nodos totales en los dos árboles (al conectar): %d nodos\n', total_nodos_encontrados);
    fprintf('3. Tiempo de optimización (RRT* Post-procesamiento doble): %.4f segundos\n', tiempo_optimizacion);
    fprintf('4. Nodos en el camino final optimizado: %d nodos\n', nodos_camino_final);
    fprintf('-------------------------------\n');

else
    disp('No se encontró conexión dentro del número máximo de iteraciones.');
end
