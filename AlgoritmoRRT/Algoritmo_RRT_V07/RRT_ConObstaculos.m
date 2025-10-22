% Algoritmo RRT con un primer obstaculos (simulacion de laberinto con obstaculos)
% Martin Luna Rogel
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

% =========================================================================
%  === PARAMETROS DEL ESPACIO Y DEL ALGORITMO ===
% =========================================================================
x_limites = [0, 100];   % Define la anchura del espacio de trabajo, de 0 a 100 en el eje X.
y_limites = [0, 100];   % Define la altura del espacio de trabajo, de 0 a 100 en el eje Y.
inicio = [10, 20];      % Es el punto de partida del algoritmo. Se usará para iniciar el "árbol" de búsqueda.
objetivo = [90, 80];    % Es el punto al que el algoritmo debe llegar.
tolerancia = 3;         % Distancia máxima al objetivo para considerar que se ha alcanzado. Si un nodo está a 5 unidades o menos, se considera un éxito.
max_iter = 7000;        % Número máximo de intentos para encontrar un camino. Si no lo encuentra, se detiene para no quedarse en un bucle infinito.
step_size = 2;        % La longitud de cada paso del algoritmo. El "árbol" crecerá 3 unidades en cada iteración.
prob_objetivo = 0.05;    % Probabilidad de muestrear el punto objetivo (5%)
grid_size = 5;         % Tamaño de la celda de la cuadricula (10x10 unidades)
prob_bias = 0.5;        % Probabilidad del 50% de usar el muestreo por densidad
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
% === CÁLCULO VECTORIZADO DEL MÍNIMO ANCHO GLOBAL ===
% =========================================================================

min_anchos = []; % Arreglo para guardar la dimensión más estrecha de cada obstáculo

% Iterar sobre cada obstáculo en el cell array
for i = 1:size(obstaculos, 1)
    x_lim = obstaculos{i, 1}; % Límites X: [X_min, X_max]
    y_lim = obstaculos{i, 2}; % Límites Y: [Y_min, Y_max]

    ancho_x = x_lim(2) - x_lim(1); % Dimensión en X
    alto_y = y_lim(2) - y_lim(1);  % Dimensión en Y

    % Encontrar la dimensión más estrecha de este obstáculo (min(ancho, alto))
    min_obs_i = min([ancho_x, alto_y]);

    % Agregar el valor al arreglo
    min_anchos = [min_anchos, min_obs_i];
end

% Encontrar el MÍNIMO GLOBAL y establecer la distancia de muestreo
min_ancho_global = min(min_anchos);
distancia_muestreo = min_ancho_global / 3; % Factor de seguridad (3)

if distancia_muestreo <= 0
    distancia_muestreo = 0.1;
end


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

    % Extraer los límites del obstáculo actual (obs_i)
    x_lim = obstaculos{i, 1}; % [X_min, X_max]
    y_lim = obstaculos{i, 2}; % [Y_min, Y_max]

    % Dibujar el obstáculo usando los límites extraídos
    fill([x_lim(1), x_lim(2), x_lim(2), x_lim(1)], ... % Coordenadas X: (min, max, max, min)
         [y_lim(1), y_lim(1), y_lim(2), y_lim(2)], ... % Coordenadas Y: (min, min, max, max)
         'k', 'FaceAlpha', 0.3);                       % 'FaceAlpha' es la transparencia del relleno. 0.3 lo hace semitransparente
end


% =========================================================================
%  === INICIALIZACIÓN DEL MAPA DE DENSIDAD ===
% =========================================================================
% Función auxiliar para convertir coordenadas (x, y) a índices de celda (columna, fila)
% Hace el mapeo de coordenadas [X_min, X_max] a índices [1, num_celdas]
coord_to_idx = @(coord) [ceil((coord(1) - x_limites(1)) / grid_size), ceil((coord(2) - y_limites(1)) / grid_size)];

% Contar el nodo de inicio en el mapa
idx_inicio = coord_to_idx(inicio);
% Ajuste de límites: Si un punto está en (0, y), queremos que vaya a (1, y)
idx_inicio(idx_inicio <= 0) = 1;

% Incrementar el contador de la celda de inicio
node_density_map(idx_inicio(2), idx_inicio(1)) = node_density_map(idx_inicio(2), idx_inicio(1)) + 1;


% =========================================================================
%  === ALGORITMO RRT ===
% =========================================================================
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
        idx_new_node = coord_to_idx(new_node);

        % Asegurar que los índices no excedan el máximo de celdas (importante para puntos en el borde superior/derecho)
        idx_new_node(1) = min(idx_new_node(1), num_celdas_x); % Asegura que Columna <= num_celdas_x
        idx_new_node(2) = min(idx_new_node(2), num_celdas_y); % Asegura que Fila <= num_celdas_y

        % También se asegura de que no haya índices <= 0 (aunque con el ajuste de la funcion, es menos probable)
        idx_new_node(idx_new_node <= 0) = 1;

        node_density_map(idx_new_node(2), idx_new_node(1)) = node_density_map(idx_new_node(2), idx_new_node(1)) + 1;


        % Dibujar la conexión
        plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], 'b');      % Dibuja una línea azul para mostrar el crecimiento del árbol.
        drawnow;        % Actualiza el gráfico en tiempo real para poder ver el proceso.

        % Verificar si se alcanzó el objetivo
        if norm(new_node - objetivo) < tolerancia   % Comprueba si el nuevo nodo está lo suficientemente cerca del objetivo. `norm` calcula la distancia entre los dos puntos.
            disp('Objetivo alcanzado!');            % Muestra un mensaje en la ventana de comandos.

            % Reconstruir la trayectoria desde el nodo final al inicio
            path = new_node;                    % Se inicia la ruta con el nodo que llegó al objetivo.
            p = size(nodes,1);                  % `p` se convierte en el índice del último nodo (el que llegó al objetivo).
            while parent(p) ~= 0                % Bucle para ir hacia atrás, de nodo en nodo, hasta llegar al inicio (el nodo cuyo padre es 0).
                p = parent(p);                  % `p` se actualiza con el índice del nodo padre.
                path = [nodes(p,:); path];      % Añade el nodo padre al inicio de la ruta.
            end

            plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2); % Dibuja la ruta final en rojo y con una línea más gruesa.
            plot(path(:,1), path(:,2), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'black');  % Remarcar los nodos por los que paso el camino a el nodo final
            % Llama a la función que grafica la distribución

            break; % Detiene el bucle 'for' porque la misión se ha completado.
        end
    end
end
