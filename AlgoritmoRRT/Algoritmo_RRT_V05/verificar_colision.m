% =========================================================================
% === FUNCION AUXILIAR PARA VERIFICAR COLISIONES ===
% =========================================================================

function hay_colision = verificar_colision(node1, node2, obstaculos, distancia_muestreo)
% Revisa la colisión del segmento contra TODOS los obstáculos en el cell array.

distancia_total = norm(node2 - node1);
num_subdivisiones = ceil(distancia_total / distancia_muestreo);

if num_subdivisiones == 0
    num_subdivisiones = 1;
end

% Iterar sobre cada obstáculo
for i = 1:size(obstaculos, 1)
    x_lim = obstaculos{i, 1}; % [X_min, X_max]
    y_lim = obstaculos{i, 2}; % [Y_min, Y_max]

    x_min = x_lim(1); x_max = x_lim(2);
    y_min = y_lim(1); y_max = y_lim(2);

    % Muestrear a lo largo del segmento de línea
    for j = 0:num_subdivisiones
        punto_intermedio = node1 + (j / num_subdivisiones) * (node2 - node1);

        % Se verifica si el punto intermedio está dentro del obstáculo actual
        if (punto_intermedio(1) >= x_min && punto_intermedio(1) <= x_max && ...
            punto_intermedio(2) >= y_min && punto_intermedio(2) <= y_max)
            hay_colision = true;
            return;
        end
    end
end

hay_colision = false;
end
