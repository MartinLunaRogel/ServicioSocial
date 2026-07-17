function [path_coords, path_indices] = Reconstruir_Camino_RRT_V20(idx_union, nodes, parent)
    path_indices = [];
    p = idx_union;
    while parent(p) ~= 0
        path_indices = [p; path_indices];
        p = parent(p);
    end
    path_indices = [1; path_indices];
    path_coords = nodes(path_indices, :);
end
