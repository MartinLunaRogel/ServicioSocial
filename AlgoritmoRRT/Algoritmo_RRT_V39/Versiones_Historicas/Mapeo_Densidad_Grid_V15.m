function [col_idx, row_idx] = Mapeo_Densidad_Grid_V15(coord, x_limites, y_limites, grid_size)
    x = coord(1);
    y = coord(2);

    col_idx = ceil((x - x_limites(1)) / grid_size);
    row_idx = ceil((y - y_limites(1)) / grid_size);

    col_idx = max(1, col_idx);
    row_idx = max(1, row_idx);
end
