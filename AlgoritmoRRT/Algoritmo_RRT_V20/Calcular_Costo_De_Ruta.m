% Calcular_Costo_De_Ruta.m
% =========================================================================
% === FUNCION AUXILIAR PARA CALCULAR EL COSTO TOTAL DEL CAMINO ===
%
% Descripción: Calcula la distancia acumulada (costo) desde un nodo dado
% hasta el nodo raíz (parent == 0) de su respectivo árbol.
% =========================================================================

function total_cost = Calcular_Costo_De_Ruta(idx_nodo, nodes, parent)
    total_cost = 0;
    actual_idx = idx_nodo;

    % Se itera hacia atrás, desde el nodo hasta el inicio (parent(1) = 0)
    while parent(actual_idx) ~= 0
        padre_idx = parent(actual_idx);
        % Calcula la distancia del segmento actual
        distancia_segmento = norm(nodes(actual_idx, :) - nodes(padre_idx, :));
        total_cost = total_cost + distancia_segmento;
        actual_idx = padre_idx;
    end
end
