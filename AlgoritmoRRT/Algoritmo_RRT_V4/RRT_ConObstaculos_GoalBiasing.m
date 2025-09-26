% Algoritmo RRT con un primer obstaculo (barra vertical)
% Martin Luna Rogel
%
% Descripción:
%   Algoritmo RRT (Rapidly-exploring Random Tree).
%   El algoritmo genera un árbol de nodos aleatorios que crece desde un punto
%   inicial hasta alcanzar un punto objetivo dentro de un área especifica definida por dos limites
%   evadiendo los obstaculos que hay en el mismo espacio de trabajo entre el punto inicial y el punto final.
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

% Parámetros del espacio y del algoritmo
x_lim = [0, 100];       % Define la anchura del espacio de trabajo, de 0 a 100 en el eje X.
y_lim = [0, 100];       % Define la altura del espacio de trabajo, de 0 a 100 en el eje Y.
inicio = [10, 10];      % Es el punto de partida del algoritmo. Se usará para iniciar el "árbol" de búsqueda.
objetivo = [90, 90];    % Es el punto al que el algoritmo debe llegar.
tolerancia = 4;         % Distancia máxima al objetivo para considerar que se ha alcanzado. Si un nodo está a 5 unidades o menos, se considera un éxito.
max_iter = 2000;        % Número máximo de intentos para encontrar un camino. Si no lo encuentra, se detiene para no quedarse en un bucle infinito.
step_size = 3;          % La longitud de cada paso del algoritmo. El "árbol" crecerá 3 unidades en cada iteración.
prob_objetivo = 0.05;   % Probabilidad de muestrear el punto objetivo (5%)

% Definir obstáculo: barra vertical en el centro
x_obs = 50;        % Posición del centro del obstáculo en el eje X.
w_obs = 0.5;         % El ancho del obstáculo.
y_obs = [20 80];   % Define la altura del obstáculo, yendo desde y=20 hasta y=80.

% Inicialización de estructuras
nodes = inicio;      % Se crea una lista para guardar todos los puntos del "árbol". El primer punto es el 'inicio'.
parent = 0;          % Se crea una lista para guardar el "padre" de cada punto. Se usará para trazar la ruta final.
parent(1) = 0;       % El primer punto no tiene padre, por lo que se le asigna un valor especial, en este caso 0.

% Inicializa un arreglo para guardar los valores aleatorios del random
valores_x_aleatorios = [];  % Valores de las coordenadas x
valores_y_aleatorios = [];  % Valores de las coordenadas y

% Dibujar el espacio
figure;         % Abre una nueva ventana de gráfico.
hold on;        % Permite dibujar múltiples elementos en la misma ventana, como el punto de inicio, el obstáculo y las líneas del árbol.
grid on;        % Muestra una cuadrícula.
xlim(x_lim);    % Fija el límite horizontal del gráfico.
ylim(y_lim);    % Fija el límite vertical del gráfico.
plot(inicio(1), inicio(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');         % Dibuja el punto de inicio en el gráfico como un círculo verde.
plot(objetivo(1), objetivo(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');     % Dibuja el punto de llegada como un círculo rojo.
title('Algoritmo RRT con obstaculo');   % Añade un título en la parte superior del gráfico.
xlabel('X');    % Etiqueta el eje X.
ylabel('Y');    % Etiqueta el eje Y.

% Dibujar obstáculo
% La función `fill` dibuja un poligono y la rellena. Para dibujar un rectángulo, se necesitan las coordenadas de sus cuatro esquinas.
fill([x_obs-w_obs/2, x_obs+w_obs/2, x_obs+w_obs/2, x_obs-w_obs/2], ... % Vector de coordenadas X. Calcula los bordes izquierdo y derecho del obstáculo a partir de su centro y ancho.
     [y_obs(1), y_obs(1), y_obs(2), y_obs(2)], ...                     % Vector de coordenadas Y. Usa los límites de altura del obstáculo.
     'k', ...                                                          % 'k' es el código de color para el contorno.
     'FaceAlpha', 0.3);                                                % 'FaceAlpha' es la transparencia del relleno. 0.3 lo hace semitransparente.


% Algoritmo RRT
% El bucle principal donde se ejecuta la lógica de expansión del árbol.
for i = 1:max_iter % El bucle se repite hasta el maximo de iteraciones definido. Si encuentra la solución antes, se detiene.

    % --------------------------------------------------------------------------
    % Generar un punto aleatorio en el espacio
    if rand < prob_objetivo % Si el número aleatorio es menor que la probabilidad...
        rand_point = objetivo;  % ...el punto aleatorio es el punto objetivo.
    else
        rand_point = [rand*(x_lim(2)-x_lim(1)), ...
                      rand*(y_lim(2)-y_lim(1))]; % Si no, genera el punto aleatorio normal.
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
    direction = (rand_point - nearest_node);           % direction: Vector que apunta desde el nodo más cercano hacia el punto aleatorio.
    direction = direction / norm(direction);           % Se normaliza el vector. Esto lo convierte en un vector de "longitud" 1. Se hace para asegurar que el siguiente paso sea de la distancia correcta (`step_size`).
    new_node = nearest_node + step_size * direction;   % new_node: Calcula las coordenadas del nuevo punto. Es el nodo más cercano más un paso de longitud definida "Step_size" en la dirección correcta.

    % --------------------------------------------------------------------------
    % Verificar colision con el obstaculo
    if ~verificar_colision(nearest_node, new_node, x_obs, w_obs, y_obs) % Se llama a la funcion verificar_colosion y se mandan los valores de: el nodo mas cercano, el nuevo nodo y valores del obstaculo

        % Si no hay colisión, el nuevo nodo se añade al árbol.
        nodes = [nodes; new_node];      % Agrega el nuevo nodo a la lista de todos los nodos.
        parent(end+1) = idx;            % Registra quién es el "padre" de este nuevo nodo (el nodo más cercano que se encontró).

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
