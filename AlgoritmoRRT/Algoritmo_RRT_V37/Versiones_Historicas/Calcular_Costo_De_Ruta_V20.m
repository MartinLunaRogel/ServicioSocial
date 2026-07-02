function total_cost = Calcular_Costo_De_Ruta_V20(idx_nodo, nodes, parent)
    total_cost = 0;
    actual_idx = idx_nodo;
    while parent(actual_idx) ~= 0
        padre_idx = parent(actual_idx);
        distancia_segmento = norm(nodes(actual_idx, :) - nodes(padre_idx, :));
        total_cost = total_cost + distancia_segmento;
        actual_idx = padre_idx;
    end
end
