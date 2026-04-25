% Optimizar_Ruta_Final.m
% =========================================================================
% === FUNCION DE OPTIMIZACIÓN (Path Shortcutting / Suavizado) ===
%
% Descripción: Toma las coordenadas de la ruta inicial y "corta camino"
% uniendo con líneas rectas los nodos más lejanos posibles sin chocar.
% =========================================================================

function path_final_coords = Optimizar_Ruta_Final(path_coords, obstaculos, Verificar_Colision_Segmento, distancia_muestreo)
    path_optimizado = path_coords;

    % Definimos cuántos "intentos" de atajos hará. 400 es rapidísimo y muy efectivo.
    max_intentos = 400;

    for k = 1:max_intentos
        num_nodos = size(path_optimizado, 1);
        if num_nodos <= 2
            break; % Ya es una línea recta, no se puede optimizar más
        end

        % Elegir dos índices aleatorios separados por al menos 1 nodo
        idx1 = randi(num_nodos - 2);
        idx2 = randi([idx1 + 2, num_nodos]);

        nodo_A = path_optimizado(idx1, :);
        nodo_B = path_optimizado(idx2, :);

        % Si hay línea de visión directa entre A y B...
        if ~Verificar_Colision_Segmento(nodo_A, nodo_B, obstaculos, distancia_muestreo)
            % Opcional: Descomenta estas dos líneas si quieres ver los atajos verdes en vivo,
            % pero dejarlas comentadas hará que se ejecute al instante.
            % plot([nodo_A(1), nodo_B(1)], [nodo_A(2), nodo_B(2)], 'g--', 'LineWidth', 1.5);
            % drawnow;

            % ...eliminamos todos los nodos intermedios ¡Atajo exitoso!
            path_optimizado(idx1+1 : idx2-1, :) = [];
        end
    end

    path_final_coords = path_optimizado;
end
