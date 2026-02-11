% =========================================================================
% === FUNCION AUXILIAR PARA VERIFICAR COLISIONES (Poligonal) ===
%
% Descripción: Muestrea un segmento de línea (node1 a node2) a una
% granularidad de 'distancia_muestreo' y verifica si alguno de los puntos
% muestreados cae dentro de alguno de los obstáculos poligonales.
% =========================================================================

function hay_colision = Verificar_Colision_Segmento(node1, node2, obstaculos, distancia_muestreo)
% Define la función con los nodos (inicio y fin del segmento), los obstáculos y la granularidad de muestreo.

distancia_total = norm(node2 - node1);      % Calcula la longitud total del segmento (distancia euclidiana) a verificar.
num_subdivisiones = ceil(distancia_total / distancia_muestreo); % Determina cuántos pasos se necesitan para muestrear el segmento con la granularidad definida.

if num_subdivisiones == 0                   % Condición de seguridad: si la distancia es cero o muy pequeña,
    num_subdivisiones = 1;                  % se asegura al menos una subdivisión para evitar división por cero o errores.
end

% Pre-alocación de memoria para los puntos de prueba a lo largo del segmento.
puntos_de_prueba_x = zeros(num_subdivisiones + 1, 1); % Inicializa un vector de ceros para las coordenadas X de los puntos muestreados.
puntos_de_prueba_y = zeros(num_subdivisiones + 1, 1); % Inicializa un vector de ceros para las coordenadas Y de los puntos muestreados.

% Bucle para generar los puntos de prueba.
for j = 0:num_subdivisiones                 % Itera desde 0 (node1) hasta el número total de subdivisiones (node2).
    punto_intermedio = node1 + (j / num_subdivisiones) * (node2 - node1); % Calcula las coordenadas (x,y) de cada punto intermedio a lo largo del segmento.
    puntos_de_prueba_x(j+1) = punto_intermedio(1); % Almacena la coordenada X del punto intermedio en el vector de pruebas.
    puntos_de_prueba_y(j+1) = punto_intermedio(2); % Almacena la coordenada Y del punto intermedio en el vector de pruebas.
end

% Bucle de verificación contra todos los obstáculos.
for i = 1:size(obstaculos, 1)               % Itera a través de cada obstáculo definido en el cell array.

    vertices = obstaculos{i, 1};            % Extrae la matriz Nx2 de vértices [X, Y] del obstáculo actual.

    V_x = vertices(:, 1);                   % Obtiene el vector de coordenadas X de los vértices del polígono.
    V_y = vertices(:, 2);                   % Obtiene el vector de coordenadas Y de los vértices del polígono.

    in = inpolygon(puntos_de_prueba_x, puntos_de_prueba_y, V_x, V_y); % Usa la función inpolygon para verificar si algún punto de prueba está DENTRO del polígono.
                                                                    % 'in' es un vector lógico (1 si está dentro, 0 si está fuera).

    if any(in)                              % Verifica si AL MENOS UN punto de prueba resultó estar dentro del polígono (si 'any(in)' es verdadero).
        hay_colision = true;                % Si hay un punto dentro, establece la bandera de colisión como verdadera.
        return;                             % Detiene la función inmediatamente, ya que se encontró una colisión.
    end
end

hay_colision = false;                       % Si el bucle termina sin encontrar colisiones, la bandera permanece falsa.
end                                         % Fin de la función.
