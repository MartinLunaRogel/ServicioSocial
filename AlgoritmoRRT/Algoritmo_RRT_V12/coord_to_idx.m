% coord_to_idx.m
% =========================================================================
% === FUNCION AUXILIAR PARA MAPEO DE COORDENADAS A INDICES DE CELDA ===
% =========================================================================

function [col_idx, row_idx] = coord_to_idx(coord, x_limites, y_limites, grid_size)
% Convierte coordenadas (x, y) a índices de celda (columna, fila)
% Hace el mapeo de coordenadas [X_min, X_max] a índices [1, num_celdas]

    x = coord(1);
    y = coord(2);

    % Calcula el índice de columna (X)
    col_idx = ceil((x - x_limites(1)) / grid_size);

    % Calcula el índice de fila (Y)
    row_idx = ceil((y - y_limites(1)) / grid_size);

    % Asegurar que el índice mínimo sea 1
    col_idx = max(1, col_idx);
    row_idx = max(1, row_idx);

    % Opcional: limitar al máximo si el punto está exactamente en el borde (aunque ceil/max ya lo gestiona bien)
    % num_celdas_x = ceil((x_limites(2) - x_limites(1)) / grid_size);
    % num_celdas_y = ceil((y_limites(2) - y_limites(1)) / grid_size);
    % col_idx = min(col_idx, num_celdas_x);
    % row_idx = min(row_idx, num_celdas_y);
end
