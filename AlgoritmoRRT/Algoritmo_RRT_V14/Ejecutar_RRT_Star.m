% Algoritmo RRT Bidireccional con un primer obstaculos
% Martin Luna Rogel%
%
% Descripción:
%   Algoritmo RRT Bidireccional (Bi-RRT).
%   Dos árboles crecen simultáneamente, uno desde el punto inicial y otro desde el objetivo.
%   El algoritmo termina cuando los dos árboles se encuentran y se conectan,
%   reduciendo el tiempo de búsqueda. Posteriormente, se aplica una
%   optimización RRT* post-procesamiento.
%
% Entradas:
%   - Ninguna.
%
% Salidas:
%   - Visualización grafica:
%       • Punto inicial (verde).
%       • Punto final (rojo).
%       • Expansión del árbol T1 (líneas azules).
%       • Expansión del árbol T2 (líneas magenta).
%       • Conexión (línea verde gruesa).
%       • Camino final encontrado y optimizado (línea roja ).
clc; clear; close all;
pkg load statistics;
pkg load geometry;

% =========================================================================
%  === PARAMETROS DEL ESPACIO Y DEL ALGORITMO ===
% =========================================================================
x_limites = [0, 100];   % Define la anchura del espacio de trabajo, de 0 a 100 en el eje X.
y_limites = [0, 100];   % Define la altura del espacio de trabajo, de 0 a 100 en el eje Y.
inicio = [10, 20];      % Es el punto de partida del algoritmo. Se usará para iniciar el "árbol" de búsqueda.
objetivo = [90, 80];    % Es el punto al que el algoritmo debe llegar.
tolerancia = 3;         % Distancia máxima al objetivo para considerar que se ha alcanzado. Si un nodo está a 5 unidades o menos, se considera un éxito.
max_iter = 7000;        % Número máximo de intentos para encontrar un camino. Si no lo encuentra, se detiene para no quedarse en un bucle infinito.
step_size = 3;        % La longitud de cada paso del algoritmo. El "árbol" crecerá 3 unidades en cada iteración.
prob_objetivo = 0.05;    % Probabilidad de muestrear el punto objetivo (5%)
grid_size = 5;         % Tamaño de la celda de la cuadricula (10x10 unidades)
prob_bias = 0.12;        % Probabilidad del 50% de usar el muestreo por densidad
num_celdas_x = ceil((x_limites(2) - x_limites(1)) / grid_size);
num_celdas_y = ceil((y_limites(2) - y_limites(1)) / grid_size);

% Inicializar el mapa de densidad: Contiene el número de nodos en cada celda (i, j)
% Se inicializa con ceros.
node_density_map = zeros(num_celdas_y, num_celdas_x);


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
%  === INICIALIZAR LA ESTRUCTURA DE LOS DOS ÁRBOLES ===
% =========================================================================

% === ÁRBOL 1 (T1): CRECE DESDE EL INICIO ===
nodes1 = inicio;      % Se crea una lista para guardar todos los puntos del "árbol". El primer punto es el 'inicio'.
parent1 = 0;          % Se crea una lista para guardar el "padre" de cada punto. Se usará para trazar la ruta final.
parent1(1) = 0;       % El primer punto no tiene padre, por lo que se le asigna un valor especial, en este caso 0.
node_density_map1 = node_density_map; % Mapa de densidad para T1

% === ÁRBOL 2 (T2): CRECE DESDE EL OBJETIVO ===
nodes2 = objetivo;    % T2 inicia en el objetivo
parent2 = 0;
parent2(1) = 0;       % El primer punto de T2 (el objetivo) no tiene padre.
node_density_map2 = node_density_map; % Mapa de densidad para T2

% Inicializa un arreglo para guardar los valores aleatorios del random
valores_x_aleatorios = [];  % Valores de las coordenadas x
valores_y_aleatorios = [];  % Valores de las coordenadas y


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
title('Algoritmo RRT Bidireccional con obstaculos');   % Añade un título en la parte superior del gráfico.
xlabel('X');    % Etiqueta el eje X.
ylabel('Y');    % Etiqueta el eje Y.

% Dibujar los obstaculos
% Se itera sobre el cell array 'obstaculos' para dibujar cada figura
for i = 1:size(obstaculos, 1)

    % Extraer los vértices del obstáculo actual (obs_i)
    vertices = obstaculos{i, 1};

    % Dibujar el obstáculo usando los límites extraídos
    patch(vertices(:,1), vertices(:,2), 'k', 'FaceAlpha', 0.3, 'EdgeColor', 'k'); % ¡CORRECCIÓN CRÍTICA! Se añade el punto y coma final para cerrar la instrucción
end


% =========================================================================
%  === INICIALIZACIÓN DE LOS MAPAS DE DENSIDAD ===
% =========================================================================

% Contar el nodo de inicio en el mapa T1
% Llamar a la función externa Mapeo_Densidad_Grid
[col_idx_inicio, row_idx_inicio] = Mapeo_Densidad_Grid(inicio, x_limites, y_limites, grid_size);
% Los índices se invierten (Fila, Columna) en la matriz:
node_density_map1(row_idx_inicio, col_idx_inicio) = node_density_map1(row_idx_inicio, col_idx_inicio) + 1;

% Contar el nodo objetivo en el mapa T2
% Llamar a la función externa Mapeo_Densidad_Grid
[col_idx_obj, row_idx_obj] = Mapeo_Densidad_Grid(objetivo, x_limites, y_limites, grid_size);
% Los índices se invierten (Fila, Columna) en la matriz:
node_density_map2(row_idx_obj, col_idx_obj) = node_density_map2(row_idx_obj, col_idx_obj) + 1;


% =========================================================================
%  === ALGORITMO RRT BIDIRECCIONAL (CAMBIO PUNTO 4, 5, 6, 7, 8, 9) ===
% =========================================================================

% >>> INICIO DEL CONTEO DE TIEMPO PARA LA FASE DE BÚSQUEDA INICIAL (RRT) <<<
tic;

% El bucle principal donde se ejecuta la lógica de expansión del árbol.
for i = 1:max_iter % El bucle se repite hasta el maximo de iteraciones definido. Si encuentra la solución antes, se detiene.

    % --------------------------------------------------------------------------
    % === LÓGICA DE ALTERNANCIA Y ASIGNACIÓN DE VARIABLES ===
    % --------------------------------------------------------------------------
    if mod(i, 2) == 1 % Iteración impar: crece T1 (desde inicio)
        active_nodes = nodes1; active_parent = parent1; active_density_map = node_density_map1;
        passive_nodes = nodes2; passive_parent = parent2;
        current_color = 'b'; % Color para T1 (Azul)
    else % Iteración par: crece T2 (desde objetivo)
        active_nodes = nodes2; active_parent = parent2; active_density_map = node_density_map2;
        passive_nodes = nodes1; passive_parent = parent1;
        current_color = 'm'; % Color para T2 (Magenta)
    end


    % --------------------------------------------------------------------------
    % Generar un punto aleatorio en el espacio
    r_prob = rand;

    if r_prob < prob_objetivo     % 1. Muestrear el punto objetivo (Para el árbol que está creciendo)
        rand_point = objetivo;    % El punto aleatorio es el punto objetivo.

    elseif r_prob < prob_objetivo + prob_bias   % 2. Muestreo con el bias de exploracion

        % Crear una matriz de probabilidades inversas (favorables a baja densidad
        % Se usa 1/cont^2 + epsilon para dar mucho mas peso a las celdas cvacias (cont=0 0 cont =1)
        epsilon = 0.01;       % Evita division por cero y asegura que las celdas con 1 nodo tenga mas probabilidad > 0
        prob_matrix = 1 ./ (active_density_map.^2 + epsilon); % Usa el mapa del árbol activo

        % Normalizar la matriz de probabilidades
        prob_vector = prob_matrix(:); % Convierte la matriz en un vector
        prob_vector = prob_vector / sum(prob_vector);

        % Seleccionar una celda de la cuadrícula basándose en estas probabilidades
        selected_cell_idx = randsample(numel(prob_vector), 1, true, prob_vector);

        % Convertir el índice lineal (de randsample) a índices de fila y columna
        [row_idx, col_idx] = ind2sub(size(active_density_map), selected_cell_idx);

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

    % Guarda las coordenadas X e Y en los arreglos
    valores_x_aleatorios = [valores_x_aleatorios; rand_point(1)];   % valoes aleatorios de la coordenada X
    valores_y_aleatorios = [valores_y_aleatorios; rand_point(2)];   % valoes aleatorios de la coordenada X

    % --------------------------------------------------------------------------
    % Buscar el nodo más cercano en el árbol activo
    % Se encuentra el nodo existente en el árbol que está más cerca del punto aleatorio.
    dif = active_nodes - rand_point;           % Calcula la distancia entre el punto aleatorio y CADA uno de los nodos que ya existen en el árbol.
    distancias = sqrt(sum(dif.^2, 2));         % Calcula la distancia en línea recta (euclidiana) de cada nodo al punto aleatorio.
    [~, idx_active] = min(distancias);         % Encuentra la posición (el "índice") del nodo con la distancia más corta.
    nearest_node = active_nodes(idx_active, :);% nearest_node: Usa la posición (`idx_active`) para obtener las coordenadas exactas de ese nodo más cercano.

    % --------------------------------------------------------------------------
    % Avanzar desde el nodo más cercano hacia el punto aleatorio
    % Se crea un nuevo nodo en el camino al punto aleatorio, a una distancia fija.
    direction = (rand_point - nearest_node);

    % Comprobar la magnitud de la dirección para evitar la división por cero (NaN)
    distancia_a_rand = norm(direction); % Calcula la distancia entre el más cercano y el aleatorio

    if distancia_a_rand < 1e-6
        continue;
    end

    direction = direction / distancia_a_rand;           % Se normaliza el vector.
    new_node = nearest_node + step_size * direction;   % new_node: Calcula las coordenadas del nuevo punto.

    % --------------------------------------------------------------------------
    % Verificar colision con el obstaculo
    if ~Verificar_Colision_Segmento(nearest_node, new_node, obstaculos, distancia_muestreo) % Se llama a la funcion verificar_colosion

        % Si no hay colisión, el nuevo nodo se añade al árbol activo.
        active_nodes = [active_nodes; new_node];      % Agrega el nuevo nodo a la lista de todos los nodos activos.
        new_node_idx_active = size(active_nodes, 1);  % Índice del nodo recién añadido en el árbol activo.
        active_parent(end+1) = idx_active;            % Registra quién es el "padre" de este nuevo nodo.

        % Actualizar el Mapa de Densidad con el nuevo nodo
        % Llamar a la funcion externa Mapeo_Densidad_Grid
        [col_idx_new_node, row_idx_new_node] = Mapeo_Densidad_Grid(new_node, x_limites, y_limites, grid_size);
        % Los índices se invierten (Fila, Columna) en la matriz:
        active_density_map(row_idx_new_node, col_idx_new_node) = active_density_map(row_idx_new_node, col_idx_new_node) + 1;

        % Dibujar la conexión del árbol activo
        plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], current_color);
        drawnow;        % Actualiza el gráfico en tiempo real para poder ver el proceso.


        % ----------------------------------------------------------------------
        % === ACTUALIZAR ESTRUCTURAS DE ÁRBOLES (CRÍTICO) ===
        % ----------------------------------------------------------------------
        % Guardar los cambios del árbol activo en las variables permanentes.
        if mod(i, 2) == 1 % T1 creció
            nodes1 = active_nodes; parent1 = active_parent; node_density_map1 = active_density_map;
        else % T2 creció
            nodes2 = active_nodes; parent2 = active_parent; node_density_map2 = active_density_map;
        end


        % ----------------------------------------------------------------------
        % === VERIFICAR CONEXIÓN CON EL ÁRBOL PASIVO (CAMBIO PUNTO 10) ===
        % ----------------------------------------------------------------------

        % Buscar el nodo más cercano del árbol activo (new_node) al árbol pasivo
        dif_passive = passive_nodes - new_node;
        distancias_passive = sqrt(sum(dif_passive.^2, 2));
        [min_dist, idx_passive] = min(distancias_passive);
        nearest_passive_node = passive_nodes(idx_passive, :);

        % Si la distancia mínima es menor que el paso (o un valor similar), se considera conectado
        if min_dist < step_size

            % Verificar colisión en el segmento de conexión final antes de fusionar
            if ~Verificar_Colision_Segmento(new_node, nearest_passive_node, obstaculos, distancia_muestreo)

                % >>> FIN DEL CONTEO DE TIEMPO PARA LA BÚSQUEDA INICIAL <<<
                tiempo_busqueda_inicial = toc;
                disp('¡Conexión entre árboles encontrada!');

                % ----------------------------------------------------------------------
                % === FUSIÓN DE ÁRBOLES EN ESTRUCTURAS ÚNICAS (CAMBIO PUNTO 11) ===
                % Esto es para que Optimizar_Ruta_Final funcione con 'nodes' y 'parent'
                % ----------------------------------------------------------------------

                % Índice donde comienza el Árbol 2 en el vector 'nodes' fusionado
                start_index_t2 = size(nodes1, 1) + 1;

                % 1. Fusionar nodes (T1 + T2)
                nodes = [nodes1; nodes2];
                total_nodos_encontrados = size(nodes, 1);

                % 2. Mapear y Fusionar parent
                % T2 se construye hacia atrás (del objetivo al inicio). Para que la función
                % de reconstrucción de ruta funcione, debemos asegurarnos que 'parent' vaya de Inicio a Objetivo.
                parent2_shifted = parent2;
                % Desplazar los índices de los padres de T2 (excepto el 0)
                parent2_shifted(parent2_shifted ~= 0) = parent2_shifted(parent2_shifted ~= 0) + (start_index_t2 - 1);

                parent = [parent1, parent2_shifted];


                % 3. Conectar el enlace central
                % Dibujar la conexión
                plot([new_node(1), nearest_passive_node(1)], [new_node(2), nearest_passive_node(2)], 'g-', 'LineWidth', 2);
                drawnow;


                if mod(i, 2) == 1 % T1 creció y se conectó a T2
                    % new_node (en T1) -> nearest_passive_node (en T2)
                    % El padre del nodo de T2 se convierte en el índice del nodo de T1.
                    connection_idx_t2_in_total = start_index_t2 + idx_passive - 1;
                    parent(connection_idx_t2_in_total) = new_node_idx_active; % new_node_idx_active es el índice de new_node en T1.

                    % El camino final es desde T1(1) hasta T2(1)
                    idx_end = start_index_t2; % El nodo objetivo (nodes2(1)) es el fin del camino.

                else % T2 creció y se conectó a T1
                    % new_node (en T2) -> nearest_passive_node (en T1)
                    % El padre del nodo de T1 se convierte en el índice del nodo de T2.
                    connection_idx_t2_in_total = start_index_t2 + new_node_idx_active - 1; % new_node_idx_active es el índice de new_node en T2.
                    parent(idx_passive) = connection_idx_t2_in_total; % idx_passive es el índice del nodo en T1.

                    % El camino final es desde T1(1) hasta T2(1)
                    idx_end = start_index_t2; % El nodo objetivo (nodes2(1)) es el fin del camino.
                end


                % ----------------------------------------------------------------------
                % === 1. RECONSTRUIR EL CAMINO INICIAL (ÍNDICES) ===
                % ----------------------------------------------------------------------
                path_indices = [];
                p = idx_end; % p se convierte en el índice del nodo objetivo (T2(1)) en el vector 'nodes' fusionado.

                while parent(p) ~= 0
                    path_indices = [p; path_indices]; % Guarda el índice del nodo en la lista
                    p = parent(p);
                end
                path_indices = [1; path_indices]; % Añadir el nodo de inicio (índice 1) al principio (T1(1))

                % ----------------------------------------------------------------------
                % === 2. OBTENER COORDENADAS DEL CAMINO INICIAL ===
                % ----------------------------------------------------------------------
                path_coords = nodes(path_indices, :);

                % ----------------------------------------------------------------------
                % === 3. DIBUJAR CAMINO INICIAL ===
                % ----------------------------------------------------------------------
                plot(path_coords(:,1), path_coords(:,2), 'r-', 'LineWidth', 2); % Dibuja la ruta final en rojo
                plot(path_coords(:,1), path_coords(:,2), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'black'); % Remarcar los nodos

                % ----------------------------------------------------------------------
                % === FASE DE OPTIMIZACIÓN RRT* (POST-PROCESAMIENTO) ===
                % La llamada no necesita cambios ya que 'nodes', 'parent' y 'path_indices'
                % ya fueron fusionados en los pasos anteriores.
                % ----------------------------------------------------------------------
                disp('Iniciando optimización del camino (RRT* post-procesamiento)...');

                % >>> INICIO DEL CONTEO DE TIEMPO PARA LA OPTIMIZACIÓN <<<
                tic;

                % Llamar a la función de optimización
                new_parent = Optimizar_Ruta_Final(nodes, parent, path_indices, obstaculos, @Verificar_Colision_Segmento, distancia_muestreo);

                % >>> FIN DEL CONTEO DE TIEMPO PARA LA OPTIMIZACIÓN <<<
                tiempo_optimizacion = toc;

                % ----------------------------------------------------------------------
                % === RECONSTRUIR Y DIBUJAR EL CAMINO FINAL OPTIMIZADO ===
                % ----------------------------------------------------------------------

                % **PASO CRÍTICO: BORRAR DIBUJOS PREVIOS**
                % 1. Borrar la línea de trayectoria del camino inicial (LineWidth=2) y las líneas temporales amarillas.
                h_lines = findobj(gca, 'Type', 'line', 'LineWidth', 2);
                delete(h_lines);
                % También borramos las líneas temporales amarillas de la optimización (y-- con LineWidth=1.5)
                h_temp_lines = findobj(gca, 'Type', 'line', 'LineWidth', 1.5);
                delete(h_temp_lines);

                % 2. Borrar los marcadores de los nodos del camino inicial (MarkerSize=4)
                h_markers = findobj(gca, 'Type', 'line', 'MarkerSize', 4);
                delete(h_markers);


                % Ahora que el dibujo del camino inicial fue borrado, se reconstruye el óptimo:

                % Reconstruir el camino final usando el vector de padres optimizado (new_parent)
                final_path_indices = [];
                p_final = idx_end; % Índice del nodo objetivo (el primer nodo de T2, ya fusionado)

                % Bucle para ir hacia atrás con la nueva estructura de padres
                while new_parent(p_final) ~= 0
                    final_path_indices = [p_final; final_path_indices];
                    p_final = new_parent(p_final);
                end
                final_path_indices = [1; final_path_indices]; % Asegurar que el nodo de inicio está al principio

                % Capturar la cantidad de nodos del camino final optimizado
                nodos_camino_final = length(final_path_indices);

                % Extraer coordenadas del camino final
                path_final_coords = nodes(final_path_indices, :);

                % 2. Dibujar la ruta final ÓPTIMA en rojo y más gruesa (la trayectoria)
                plot(path_final_coords(:,1), path_final_coords(:,2), 'r-', 'LineWidth', 2);

                % 3. Remarcar TODOS los nodos del camino final (los puntos individuales)
                plot(path_final_coords(:,1), path_final_coords(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');

                disp('Optimización finalizada. Camino más corto trazado.');

                % ==================================================================
                % === MOSTRAR MÉTRICAS DE RENDIMIENTO REQUERIDAS ===
                % ==================================================================
                fprintf('\n\n--- MÉTRICAS DE RENDIMIENTO ---\n');
                fprintf('1. Tiempo de búsqueda inicial (Bi-RRT): %.4f segundos\n', tiempo_busqueda_inicial);
                fprintf('2. Nodos totales en el árbol (al encontrar la conexión): %d nodos\n', total_nodos_encontrados);
                fprintf('3. Tiempo de optimización (RRT* Post-procesamiento): %.4f segundos\n', tiempo_optimizacion);
                fprintf('4. Nodos en el camino final optimizado: %d nodos\n', nodos_camino_final);
                fprintf('-------------------------------\n');

                break; % Detiene el bucle 'for' porque la misión se ha completado.
            end % Fin de la verificación de colisión de conexión.
        end % Fin de la verificación de conexión.
    end % Fin de la verificación de colisión del segmento.
end % Fin del bucle principal
