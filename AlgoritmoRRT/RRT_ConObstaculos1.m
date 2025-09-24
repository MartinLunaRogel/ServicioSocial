% Algoritmo RRT con un primer obstaculo (barra vertical)
% Martin Luna Rogel
%
% Descripción:
%   Algoritmo RRT (Rapidly-exploring Random Tree).
%   El algoritmo genera un árbol de nodos aleatorios que crece desde un punto
%   inicial hasta alcanzar un punto objetivo dentro de un área especifica definida por dos limites.
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
x_lim = [0, 100];       % x_lim: Define la anchura del espacio de trabajo, de 0 a 100 en el eje X.
y_lim = [0, 100];       % y_lim: Define la altura del espacio de trabajo, de 0 a 100 en el eje Y.
inicio = [10, 10];      % inicio: Es el punto de partida del algoritmo. Se usará para iniciar el "árbol" de búsqueda.
objetivo = [90, 90];    % objetivo: Es el punto al que el algoritmo debe llegar.
tolerancia = 5;         % tolerancia: Distancia máxima al objetivo para considerar que se ha alcanzado. Si un nodo está a 5 unidades o menos, se considera un éxito.
max_iter = 1000;        % max_iter: Número máximo de intentos para encontrar un camino. Si no lo encuentra en 1000 intentos, se detiene para no quedarse en un bucle infinito.
step_size = 3;          % step_size: La longitud de cada paso del algoritmo. El "árbol" crecerá 3 unidades en cada iteración.

% Definir obstáculo: barra vertical en el centro
x_obs = 50;        % x_obs: Posición del centro del obstáculo en el eje X.
w_obs = 2;         % w_obs: El ancho del obstáculo.
y_obs = [20 80];   % y_obs: Define la altura del obstáculo, yendo desde y=20 hasta y=80.

% Inicialización de estructuras
nodes = inicio;      % nodes: Se crea una lista para guardar todos los puntos del "árbol". El primer punto es el 'inicio'.
parent = 0;          % parent: Se crea una lista para guardar el "padre" de cada punto. Se usará para trazar la ruta final.
parent(1) = 0;       % El primer punto no tiene padre, por lo que se le asigna un valor especial, en este caso 0.

% Dibujar el espacio
figure;         % Abre una nueva ventana de gráfico.
hold on;        % Permite dibujar múltiples elementos en la misma ventana, como el punto de inicio, el obstáculo y las líneas del árbol.
grid on;        % Muestra una cuadrícula para ubicar mejor los puntos.
xlim(x_lim);    % xlim: Fija el límite horizontal del gráfico.
ylim(y_lim);    % ylim: Fija el límite vertical del gráfico.
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
for i = 1:max_iter % El bucle se repite hasta 1000 veces. Si encuentra la solución antes, se detiene.

    % 1. Generar un punto aleatorio en el espacio
    rand_point = [rand*(x_lim(2)-x_lim(1)), ...     % Crea un punto aleatorio [X,Y] dentro de los límites del área.
                  rand*(y_lim(2)-y_lim(1))];        % `rand` genera un número al azar entre 0 y 1, que se multiplica para que el punto caiga en el espacio de 100x100.

    % 2. Buscar el nodo más cercano
    % Se encuentra el nodo existente en el árbol que está más cerca del punto aleatorio.
    dif = nodes - rand_point;                  % dif: Calcula la distancia entre el punto aleatorio y CADA uno de los nodos que ya existen en el árbol.
    distancias = sqrt(sum(dif.^2, 2));         % distancias: Calcula la distancia en línea recta (euclidiana) de cada nodo al punto aleatorio.
    [~, idx] = min(distancias);                % idx: Encuentra la posición (el "índice") del nodo con la distancia más corta. El '~' indica que no nos interesa el valor de la distancia en sí, solo su posición.
    nearest_node = nodes(idx, :);              % nearest_node: Usa la posición (`idx`) para obtener las coordenadas exactas de ese nodo más cercano.

    % 3. Avanzar desde el nodo más cercano hacia el punto aleatorio
    % Se crea un nuevo nodo en el camino al punto aleatorio, a una distancia fija.
    direction = (rand_point - nearest_node);           % direction: Vector que apunta desde el nodo más cercano hacia el punto aleatorio.
    direction = direction / norm(direction);           % Se normaliza el vector. Esto lo convierte en un vector de "longitud" 1. Se hace para asegurar que el siguiente paso sea de la distancia correcta (`step_size`).
    new_node = nearest_node + step_size * direction;   % new_node: Calcula las coordenadas del nuevo punto. Es el nodo más cercano más un paso de 3 unidades en la dirección correcta.

    % 4. Verificar colision con el obstaculo
    if ~( ...                                                                           % Si el nuevo nodo NO está dentro del obstáculo, el código continúa.
         new_node(1) >= (x_obs - w_obs/2) && new_node(1) <= (x_obs + w_obs/2) && ...    % Se erifica si la coordenada X del nuevo nodo está dentro del ancho de la barra.
         new_node(2) >=  y_obs(1) && new_node(2) <= y_obs(2) ...                        % Esta parte verifica si la coordenada Y del nuevo nodo está dentro de la altura de la barra.
        )

        % Si no hay colisión, el nuevo nodo se añade al árbol.
        nodes = [nodes; new_node];      % Agrega el nuevo nodo a la lista de todos los nodos.
        parent(end+1) = idx;            % Registra quién es el "padre" de este nuevo nodo (el nodo más cercano que se encontró).

        % Dibujar la conexión
        plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], 'b');      % Dibuja una línea azul para mostrar el crecimiento del árbol.
        drawnow;        % Actualiza el gráfico en tiempo real para que puedas ver el proceso.

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
            break; % Detiene el bucle 'for' porque la misión se ha completado.
        end
    end
end
