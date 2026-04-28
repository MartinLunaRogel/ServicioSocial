% =========================================================================
% === FUNCION DE COLISIONES: RAY-CASTING DISCRETO (BRESENHAM) ===
%
% Descripción: Traza una línea digital entre dos nodos sobre el Mapa Discreto.
% Si algún píxel de la línea toca un 1 (obstáculo), hay colisión.
% Tiempo de ejecución: O(L) donde L es la longitud en píxeles.
% =========================================================================

function hay_colision = Verificar_Colision_Segmento(node1, node2, mapa_discreto)
    % 1. Mapeamos las coordenadas reales [0, 500] a índices de matriz [1, 501]
    x1 = round(node1(1)) + 1;
    y1 = round(node1(2)) + 1;
    x2 = round(node2(1)) + 1;
    y2 = round(node2(2)) + 1;

    % 2. Protección: Asegurar que los índices no salgan del tamaño de la matriz
    [max_filas, max_cols] = size(mapa_discreto);
    x1 = max(1, min(x1, max_cols));
    y1 = max(1, min(y1, max_filas));
    x2 = max(1, min(x2, max_cols));
    y2 = max(1, min(y2, max_filas));

    % 3. Configuración del algoritmo de Bresenham
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    sx = sign(x2 - x1);
    sy = sign(y2 - y1);
    err = dx - dy;

    x = x1;
    y = y1;

    % 4. Recorrido píxel por píxel
    while true
        % Verificamos colisión en el píxel actual.
        % IMPORTANTE: En MATLAB las matrices son (fila, columna) -> (Y, X)
        if mapa_discreto(y, x) == 1
            hay_colision = true;
            return; % Chocó, salimos inmediatamente
        end

        % Si llegamos al píxel final sin tocar ningún 1, la línea está libre
        if x == x2 && y == y2
            break;
        end

        % Calcular el siguiente píxel
        e2 = 2 * err;
        if e2 > -dy
            err = err - dy;
            x = x + sx;
        end
        if e2 < dx
            err = err + dx;
            y = y + sy;
        end
    end

    hay_colision = false; % Si el ciclo termina, la línea es completamente segura
end
