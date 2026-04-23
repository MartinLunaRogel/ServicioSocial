% Generar_Semillas_Libres.m
% =========================================================================
% === FUNCION PARA GENERAR PUNTOS ALEATORIOS FUERA DE OBSTACULOS ===
%
% Descripción: Genera un número especificado de coordenadas (x,y)
% asegurando que ninguna caiga dentro de los polígonos de los obstáculos.
% =========================================================================

function semillas = Generar_Semillas_Libres(num_semillas, x_limites, y_limites, obstaculos)
    semillas = [];
    intentos = 0; % Contador de seguridad
    max_intentos = 50000; % Evita un bucle infinito si el mapa está muy saturado

    % Repetir hasta que tengamos el número exacto de semillas requeridas
    while size(semillas, 1) < num_semillas && intentos < max_intentos
        intentos = intentos + 1;

        % 1. Generar un punto aleatorio candidato en todo el mapa
        rx = x_limites(1) + rand * (x_limites(2) - x_limites(1));
        ry = y_limites(1) + rand * (y_limites(2) - y_limites(1));
        punto_candidato = [rx, ry];

        % 2. Verificar si este punto cae dentro de algún obstáculo
        en_colision = false;
        for i = 1:size(obstaculos, 1)
            vertices = obstaculos{i, 1};

            % inpolygon evalúa si el punto (rx, ry) está dentro del polígono
            % vertices(:,1) son las X, vertices(:,2) son las Y
            dentro = inpolygon(rx, ry, vertices(:,1), vertices(:,2));

            if dentro
                en_colision = true;
                break; % Ya chocó con uno, no tiene caso revisar los demás
            end
        end

        % 3. Si el punto es seguro (no colisionó con nada), lo guardamos
        if ~en_colision
            semillas = [semillas; punto_candidato];
        end
    end

    % Mensaje de advertencia por si el espacio libre es muy pequeño
    if intentos >= max_intentos
        warning('Espacio saturado: Solo se pudieron generar %d de %d semillas seguras.', size(semillas, 1), num_semillas);
    end
end
