% calcular_costo_camino.m
% =========================================================================
% === FUNCION AUXILIAR PARA CALCULAR EL COSTO TOTAL DEL CAMINO ===
% =========================================================================

function total_cost = calcular_costo_camino(idx_nodo, nodes, parent)
    total_cost = 0;
    actual_idx = idx_nodo;

    % Se itera hacia atrás, desde el nodo hasta el inicio (parent(1) = 0)
    while parent(actual_idx) ~= 0
        padre_idx = parent(actual_idx);
        % Asumimos que 'nodes' y 'parent' están definidos en el workspace de quien llama o se pasan como argumento
        distancia_segmento = norm(nodes(actual_idx, :) - nodes(padre_idx, :));
        total_cost = total_cost + distancia_segmento;
        actual_idx = padre_idx;
    end
end
