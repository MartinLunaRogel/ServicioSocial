% =========================================================================
% === FUNCION AUXILIAR PARA VERIFICAR COLISIONES (Poligonal con Culling) ===
%
% Descripción: Muestrea un segmento de línea y verifica colisiones.
% Incluye optimización de Bounding Box para saltar cálculos costosos.
% =========================================================================

function hay_colision = Verificar_Colision_Segmento(node1, node2, obstaculos, distancia_muestreo)
% Define la función con los nodos, obstáculos y la granularidad de muestreo.

distancia_total = norm(node2 - node1);      % Calcula la longitud total del segmento a verificar.
num_subdivisiones = ceil(distancia_total / distancia_muestreo); % Determina cuántos pasos se necesitan para muestrear.

if num_subdivisiones == 0                   % Condición de seguridad para distancias nulas.
    num_subdivisiones = 1;                  % Se asegura al menos una subdivisión.
end

% Pre-alocación de memoria para los puntos de prueba.
puntos_de_prueba_x = zeros(num_subdivisiones + 1, 1); % Vector para coordenadas X de prueba.
puntos_de_prueba_y = zeros(num_subdivisiones + 1, 1); % Vector para coordenadas Y de prueba.

% Bucle para generar los puntos de prueba a lo largo del segmento.
for j = 0:num_subdivisiones                 % Itera desde el inicio hasta el fin del segmento.
    punto_intermedio = node1 + (j / num_subdivisiones) * (node2 - node1); % Calcula coordenadas intermedias.
    puntos_de_prueba_x(j+1) = punto_intermedio(1); % Guarda la X del punto.
    puntos_de_prueba_y(j+1) = punto_intermedio(2); % Guarda la Y del punto.
end

% Bucle de verificación contra todos los obstáculos.
for i = 1:size(obstaculos, 1)               % Itera a través de cada obstáculo.

    vertices = obstaculos{i, 1};            % Extrae los vértices del obstáculo actual.

    % === NUEVA OPTIMIZACIÓN: BOUNDING BOX CULLING ===
    obs_xmin = min(vertices(:,1));          % Calcula el límite izquierdo del obstáculo.
    obs_xmax = max(vertices(:,1));          % Calcula el límite derecho del obstáculo.
    obs_ymin = min(vertices(:,2));          % Calcula el límite inferior del obstáculo.
    obs_ymax = max(vertices(:,2));          % Calcula el límite superior del obstáculo.

    seg_xmin = min(node1(1), node2(1));     % Encuentra la X mínima del segmento actual.
    seg_xmax = max(node1(1), node2(1));     % Encuentra la X máxima del segmento actual.
    seg_ymin = min(node1(2), node2(2));     % Encuentra la Y mínima del segmento actual.
    seg_ymax = max(node1(2), node2(2));     % Encuentra la Y máxima del segmento actual.

    % Verifica si las "cajas" del segmento y el obstáculo ni siquiera se tocan.
    if seg_xmin > obs_xmax || seg_xmax < obs_xmin || seg_ymin > obs_ymax || seg_ymax < obs_ymin
        continue;                           % Si no hay solapamiento, salta al siguiente obstáculo sin calcular 'inpolygon'.
    end
    % ===============================================

    V_x = vertices(:, 1);                   % Coordenadas X de los vértices.
    V_y = vertices(:, 2);                   % Coordenadas Y de los vértices.

    in = inpolygon(puntos_de_prueba_x, puntos_de_prueba_y, V_x, V_y); % Verifica si los puntos están dentro del polígono.

    if any(in)                              % Si al menos un punto está dentro.
        hay_colision = true;                % Establece bandera de colisión.
        return;                             % Sale de la función inmediatamente.
    end
end

hay_colision = false;                       % Si termina sin colisiones, devuelve falso.
end                                         % Fin de la función.
