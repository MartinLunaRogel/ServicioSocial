% Algoritmo RRT con un primer obstaculos (simulacion de laberinto con obstaculos)
% Martin Luna Rogel%
%
% Descripción:
%   Algoritmo RRT (Rapidly-exploring Random Tree).
%   El algoritmo genera un árbol de nodos aleatorios que crece desde un punto
%   inicial hasta alcanzar un punto objetivo dentro de un área especifica definida por dos limites
%   evadiendo los obstaculos que hay en el mismo espacio de trabajo entre el punto inicial y el punto final.
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

run('Obstaculos_Para_RRT.m');


% =========================================================================
% === AJUSTE DE LA DISTANCIA DE MUESTREO PARA POLÍGONOS ===
% =========================================================================
% Se establece un valor fijo bajo para garantizar la detección de colisiones
% en pasajes estrechos con obstáculos poligonales.
distancia_muestreo = 0.5; % Define una distancia de muestreo baja (0.5 unidades) para la verificación de colisión


% =========================================================================
%  === INICIALIZAR LA ESTRUCTURA ===
% =========================================================================
nodes = inicio;      % Se crea una lista para guardar todos los puntos del "árbol". El primer punto es el 'inicio'.
parent = 0;          % Se crea una lista para guardar el "padre" de cada punto. Se usará para trazar la ruta final.
parent(1) = 0;       % El primer punto no tiene padre, por lo que se le asigna un valor especial, en este caso 0.

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
title('Algoritmo RRT con obstaculo');   % Añade un título en la parte superior del gráfico.
xlabel('X');    % Etiqueta el eje X.
ylabel('Y');    % Etiqueta el eje Y.

% Dibujar los obstaculos
% Se itera sobre el cell array 'obstaculos' para dibujar cada figura
for i = 1:size(obstaculos, 1)

    % Extraer los vértices del obstáculo actual (obs_i)
    vertices = obstaculos{i, 1};    % Esta línea parece correcta

    % Dibujar el obstáculo usando los límites extraídos
    patch(vertices(:,1), vertices(:,2), 'k', 'FaceAlpha', 0.3, 'EdgeColor', 'k'); % ¡CORRECCIÓN CRÍTICA! Se añade el punto y coma final para cerrar la instrucción
end


% =========================================================================
%  === INICIALIZACIÓN DEL MAPA DE DENSIDAD ===
% =========================================================================

% Contar el nodo de inicio en el mapa
% Llamar a la función externa coord_to_idx
[col_idx_inicio, row_idx_inicio] = coord_to_idx(inicio, x_limites, y_limites, grid_size);

% Incrementar el contador de la celda de inicio
% Los índices se invierten (Fila, Columna) en la matriz:
node_density_map(row_idx_inicio, col_idx_inicio) = node_density_map(row_idx_inicio, col_idx_inicio) + 1;


% =========================================================================
%  === ALGORITMO RRT ===
% =========================================================================

% >>> INICIO DEL CONTEO DE TIEMPO PARA LA FASE DE BÚSQUEDA INICIAL (RRT) <<<
tic;

% El bucle principal donde se ejecuta la lógica de expansión del árbol.
for i = 1:max_iter % El bucle se repite hasta el maximo de iteraciones definido. Si encuentra la solución antes, se detiene.

    % --------------------------------------------------------------------------
    % Generar un punto aleatorio en el espacio
    r_prob = rand;

    if r_prob < prob_objetivo     % 1. Muestrear el punto objetivo
        rand_point = objetivo;    % El punto aleatorio es el punto objetivo.

    elseif r_prob < prob_objetivo + prob_bias   % 2. Muestreo con el bias de exploracion

        % Crear una matriz de probabilidades inversas (favorables a baja densidad
        % Se usa 1/cont^2 + epsilon para dar mucho mas peso a las celdas cvacias (cont=0 0 cont =1)
        epsilon = 0.01;       % Evita division por cero y asegura que las celdas con 1 nodo tenga mas probabilidad > 0
        prob_matrix = 1 ./ (node_density_map.^2 + epsilon);

        % Normalizar la matriz de probabilidades
        prob_vector = prob_matrix(:); % Convierte la matriz en un vector
        prob_vector = prob_vector / sum(prob_vector);

        % Seleccionar una celda de la cuadrícula basándose en estas probabilidades
        selected_cell_idx = randsample(numel(prob_vector), 1, true, prob_vector);

        % Convertir el índice lineal (de randsample) a índices de fila y columna
        [row_idx, col_idx] = ind2sub(size(node_density_map), selected_cell_idx);

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
    % Buscar el nodo más cercano
    % Se encuentra el nodo existente en el árbol que está más cerca del punto aleatorio.
    dif = nodes - rand_point;                  % Calcula la distancia entre el punto aleatorio y CADA uno de los nodos que ya existen en el árbol.
    distancias = sqrt(sum(dif.^2, 2));         % Calcula la distancia en línea recta (euclidiana) de cada nodo al punto aleatorio.
    [~, idx] = min(distancias);                % Encuentra la posición (el "índice") del nodo con la distancia más corta. El '~' indica que no nos interesa el valor de la distancia en sí, solo su posición.
    nearest_node = nodes(idx, :);              % nearest_node: Usa la posición (`idx`) para obtener las coordenadas exactas de ese nodo más cercano.

    % --------------------------------------------------------------------------
    % Avanzar desde el nodo más cercano hacia el punto aleatorio
    % Se crea un nuevo nodo en el camino al punto aleatorio, a una distancia fija.
    direction = (rand_point - nearest_node);
    direction = direction / norm(direction);           % Se normaliza el vector. Esto lo convierte en un vector de "longitud" 1. Se hace para asegurar que el siguiente paso sea de la distancia correcta (`step_size`).
    new_node = nearest_node + step_size * direction;   % new_node: Calcula las coordenadas del nuevo punto. Es el nodo más cercano más un paso de longitud definida "Step_size" en la dirección correcta.

    % --------------------------------------------------------------------------
    % Verificar colision con el obstaculo
    if ~verificar_colision(nearest_node, new_node, obstaculos, distancia_muestreo) % Se llama a la funcion verificar_colosion y se mandan los valores de: el nodo mas cercano, el nuevo nodo y valores del obstaculo

        % Si no hay colisión, el nuevo nodo se añade al árbol.
        nodes = [nodes; new_node];      % Agrega el nuevo nodo a la lista de todos los nodos.
        parent(end+1) = idx;            % Registra quién es el "padre" de este nuevo nodo (el nodo más cercano que se encontró).

        % Actualizar el Mapa de Densidad con el nuevo nodo
        % Llamar a la funcion externa coord_to_idx
        [col_idx_new_node, row_idx_new_node] = coord_to_idx(new_node, x_limites, y_limites, grid_size);


        % Los índices se invierten (Fila, Columna) en la matriz:
        node_density_map(row_idx_new_node, col_idx_new_node) = node_density_map(row_idx_new_node, col_idx_new_node) + 1;

        % Dibujar la conexión
        plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], 'b');      % Dibuja una línea azul para mostrar el crecimiento del árbol.
        % Solo actualiza la gráfica cada 50 iteraciones (o el valor que elijas)
        drawnow;        % Actualiza el gráfico en tiempo real para poder ver el proceso.


        % Verificar si se alcanzó el objetivo
        if norm(new_node - objetivo) < tolerancia   % Comprueba si el nuevo nodo está lo suficientemente cerca del objetivo. `norm` calcula la distancia entre los dos puntos.

            % >>> FIN DEL CONTEO DE TIEMPO PARA LA BÚSQUEDA INICIAL <<<
            tiempo_busqueda_inicial = toc;

            disp('Objetivo alcanzado!');            % Muestra un mensaje en la ventana de comandos.

            % Capturar el número total de nodos en el árbol al momento de encontrar el objetivo
            total_nodos_encontrados = size(nodes, 1);

            % ----------------------------------------------------------------------
            % === 1. RECONSTRUIR EL CAMINO INICIAL (ÍNDICES) ===
            % ----------------------------------------------------------------------
            path_indices = [];
            p = size(nodes,1); % p se convierte en el índice del último nodo (el que llegó al objetivo).

            while parent(p) ~= 0
                path_indices = [p; path_indices]; % Guarda el índice del nodo en la lista
                p = parent(p);
            end
            path_indices = [1; path_indices]; % Añadir el nodo de inicio (índice 1) al principio

            % ----------------------------------------------------------------------
            % === 2. OBTENER COORDENADAS DEL CAMINO INICIAL ===
            % ----------------------------------------------------------------------
            path_coords = nodes(path_indices, :);

            % ----------------------------------------------------------------------
            % === 3. DIBUJAR CAMINO INICIAL (EL QUE REQUERISTE NO CAMBIAR) ===
            % ----------------------------------------------------------------------
            plot(path_coords(:,1), path_coords(:,2), 'r-', 'LineWidth', 2); % Dibuja la ruta final en rojo
            plot(path_coords(:,1), path_coords(:,2), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'black'); % Remarcar los nodos

            % Llama a la función que grafica la distribución (COMENTARIO ORIGINAL)

            % ----------------------------------------------------------------------
            % === FASE DE OPTIMIZACIÓN RRT* (POST-PROCESAMIENTO) ===
            % ----------------------------------------------------------------------
            disp('Iniciando optimización del camino (RRT* post-procesamiento)...');

            % ... (Sigue con el código de optimización)
            % ----------------------------------------------------------------------
            % === FASE DE OPTIMIZACIÓN RRT* (POST-PROCESAMIENTO) ===
            % ----------------------------------------------------------------------
            disp('Iniciando optimización del camino (RRT* post-procesamiento)...');

            % >>> INICIO DEL CONTEO DE TIEMPO PARA LA OPTIMIZACIÓN <<<
            tic;

            % Llamar a la función de optimización
            % IMPORTANTE: Debe cargar o definirse la función 'optimizar_camino_rrt_star'
            new_parent = optimizar_camino_rrt_star(nodes, parent, path_indices, obstaculos, @verificar_colision, distancia_muestreo);

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
            p_final = size(nodes,1); % Índice del nodo objetivo (el último añadido)

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
            % Usamos 'ro' solo como marcador sin línea para asegurar que se vean todos los nodos del camino óptimo.
            % Cambiamos el tamaño a 5 para que resalte más.
            plot(path_final_coords(:,1), path_final_coords(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');

            disp('Optimización finalizada. Camino más corto trazado.');

            % ==================================================================
            % === MOSTRAR MÉTRICAS DE RENDIMIENTO REQUERIDAS ===
            % ==================================================================
            fprintf('\n\n--- MÉTRICAS DE RENDIMIENTO ---\n');
            fprintf('1. Tiempo de búsqueda inicial (RRT): %.4f segundos\n', tiempo_busqueda_inicial);
            fprintf('2. Nodos totales en el árbol (al encontrar el objetivo): %d nodos\n', total_nodos_encontrados);
            fprintf('3. Tiempo de optimización (RRT* Post-procesamiento): %.4f segundos\n', tiempo_optimizacion);
            fprintf('4. Nodos en el camino final optimizado: %d nodos\n', nodos_camino_final);
            fprintf('-------------------------------\n');

            break; % Detiene el bucle 'for' porque la misión se ha completado.
        end
    end
end
