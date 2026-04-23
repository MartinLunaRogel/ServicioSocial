% Optimizar_Ruta_Final.m
% =========================================================================
% === FUNCION DE OPTIMIZACIÓN (Path Shortcutting / Suavizado) ===
%
% Descripción: Toma las coordenadas de la ruta inicial y "corta camino"
% uniendo con líneas rectas los nodos más lejanos posibles sin chocar.
% =========================================================================

function path_final_coords = Optimizar_Ruta_Final(path_coords, obstaculos, Verificar_Colision_Segmento, distancia_muestreo)

    path_optimizado = path_coords;
    i = 1;

    % Recorremos la ruta desde el inicio hasta el final
    while i < size(path_optimizado, 1) - 1
        mejor_salto = i + 1;

        % Buscar el nodo más lejano HACIA ADELANTE al que podamos saltar en línea recta
        for j = size(path_optimizado, 1):-1:(i + 2)
            nodo_A = path_optimizado(i, :);
            nodo_B = path_optimizado(j, :);

            % Si podemos trazar una línea recta entre A y B sin chocar con nada...
            if ~Verificar_Colision_Segmento(nodo_A, nodo_B, obstaculos, distancia_muestreo)
                mejor_salto = j;

                % Dibujar la línea verde punteada temporal para ver el atajo en vivo
                plot([nodo_A(1), nodo_B(1)], [nodo_A(2), nodo_B(2)], 'g--', 'LineWidth', 1.5);
                drawnow;
                break; % Encontramos el atajo máximo, no necesitamos buscar saltos más cortos
            end
        end

        % Si encontramos un atajo (nos saltamos nodos intermedios), los eliminamos de la lista
        if mejor_salto > i + 1
            path_optimizado((i+1):(mejor_salto-1), :) = [];
        end

        % Avanzamos al siguiente nodo de la nueva ruta suavizada
        i = i + 1;
    end

    path_final_coords = path_optimizado;
end
