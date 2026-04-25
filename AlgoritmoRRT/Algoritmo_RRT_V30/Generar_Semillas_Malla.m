% Generar_Semillas_Malla.m
% =========================================================================
% === FUNCION PARA GENERAR PUNTOS ESTRATEGICOS EN FORMA DE MALLA ===
% =========================================================================

function semillas = Generar_Semillas_Malla(x_limites, y_limites, obstaculos, separacion)
    semillas = [];

    % 1. Crear los ejes de la cuadrícula basados en la separación deseada
    x_coords = x_limites(1) : separacion : x_limites(2);
    y_coords = y_limites(1) : separacion : y_limites(2);

    % 2. Barrer todo el mapa intersección por intersección
    for i = 1:length(x_coords)
        for j = 1:length(y_coords)
            rx = x_coords(i);
            ry = y_coords(j);

            % 3. Verificar si esta intersección cae dentro de un edificio
            en_colision = false;
            for k = 1:size(obstaculos, 1)
                vertices = obstaculos{k, 1};

                % Evalúa si el punto está dentro del polígono
                dentro = inpolygon(rx, ry, vertices(:,1), vertices(:,2));

                if dentro
                    en_colision = true;
                    break; % Chocó, pasamos al siguiente punto
                end
            end

            % 4. Si es seguro (cayó en la calle), lo agregamos a las semillas
            if ~en_colision
                semillas = [semillas; rx, ry];
            end
        end
    end
end
