% =========================================================================
% === FUNCION AUXILIAR PARA VERIFICAR COLISIONES (Intersección de Líneas) ===
%
% Descripción: Verifica si el segmento entre node1 y node2 intersecta o toca
% alguna de las aristas (paredes) de los obstáculos.
% =========================================================================

function hay_colision = Verificar_Colision_Segmento(node1, node2, obstaculos, ~)
% Nota: Se mantiene el cuarto argumento (distancia_muestreo) para no romper
% la compatibilidad con las llamadas desde otros archivos, pero ya no se usa.

    % Bucle de verificación contra todos los obstáculos.
    for i = 1:size(obstaculos, 1)
        vertices = obstaculos{i, 1}; % Extrae los vértices del obstáculo.
        num_vertices = size(vertices, 1);

        % === OPTIMIZACIÓN: BOUNDING BOX CULLING (Se mantiene por eficiencia) ===
        obs_xmin = min(vertices(:,1)); obs_xmax = max(vertices(:,1));
        obs_ymin = min(vertices(:,2)); obs_ymax = max(vertices(:,2));
        seg_xmin = min(node1(1), node2(1)); seg_xmax = max(node1(1), node2(1));
        seg_ymin = min(node1(2), node2(2)); seg_ymax = max(node1(2), node2(2));

        if seg_xmin > obs_xmax || seg_xmax < obs_xmin || seg_ymin > obs_ymax || seg_ymax < obs_ymin
            continue; % Las cajas no se tocan, pasamos al siguiente obstáculo.
        end
        % =====================================================================

        % Verificamos la intersección del segmento con cada arista del polígono
        for j = 1:num_vertices
            % Definir los puntos de la arista actual (pared del obstáculo)
            v1 = vertices(j, :);
            if j < num_vertices
                v2 = vertices(j+1, :);
            else
                v2 = vertices(1, :); % La última arista conecta el último vértice con el primero.
            end

            % Lógica de Intersección de Segmentos (Basada en orientaciones)
            if Segments_Intersect(node1, node2, v1, v2)
                hay_colision = true; % ¡Se detectó un cruce o un toque!
                return; % Salimos de inmediato por eficiencia.
            end
        end
    end

    hay_colision = false; % Si revisó todas las aristas y no hubo cruces.
end

% --- Función Interna: Prueba de Intersección de Segmentos ---
function intersecta = Segments_Intersect(A, B, C, D)
    % Determina si el segmento AB intersecta al segmento CD

    % Función auxiliar para la orientación de tres puntos (Cross Product)
    % 0 -> Colineal, 1 -> Horario, 2 -> Anti-horario
    function o = orientation(p, q, r)
        val = (q(2) - p(2)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(2) - q(2));
        if abs(val) < 1e-9, o = 0; return; end % Colineales
        if val > 0, o = 1; else o = 2; end
    end

    % Casos de orientación general
    o1 = orientation(A, B, C);
    o2 = orientation(A, B, D);
    o3 = orientation(C, D, A);
    o4 = orientation(C, D, B);

    % Caso General: Las orientaciones son distintas
    if (o1 ~= o2) && (o3 ~= o4)
        intersecta = true;
        return;
    end

    % Casos Especiales: El punto toca exactamente la línea (colinealidad)
    function on = onSegment(p, q, r)
        on = r(1) <= max(p(1), q(1)) && r(1) >= min(p(1), q(1)) && ...
             r(2) <= max(p(2), q(2)) && r(2) >= min(p(2), q(2));
    end

    if (o1 == 0 && onSegment(A, B, C)), intersecta = true; return; end
    if (o2 == 0 && onSegment(A, B, D)), intersecta = true; return; end
    if (o3 == 0 && onSegment(C, D, A)), intersecta = true; return; end
    if (o4 == 0 && onSegment(C, D, B)), intersecta = true; return; end

    intersecta = false;
end
