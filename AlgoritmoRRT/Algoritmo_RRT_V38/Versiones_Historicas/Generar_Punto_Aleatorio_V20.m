function rand_point = Generar_Punto_Aleatorio_V20(other_nodes, current_density_map, x_limites, y_limites, grid_size, prob_objetivo, prob_bias)
    r_prob = rand;
    if r_prob < prob_objetivo
        rand_point = other_nodes(1, :);
    elseif r_prob < prob_objetivo + prob_bias
        epsilon = 0.01;
        prob_matrix = 1 ./ (current_density_map.^2 + epsilon);
        prob_vector = prob_matrix(:);
        prob_vector = prob_vector / sum(prob_vector);
        selected_cell_idx = randsample(numel(prob_vector), 1, true, prob_vector);
        [row_idx, col_idx] = ind2sub(size(current_density_map), selected_cell_idx);

        x_min_cell = (col_idx - 1) * grid_size + x_limites(1);
        y_min_cell = (row_idx - 1) * grid_size + y_limites(1);
        rand_x = x_min_cell + rand * grid_size;
        rand_y = y_min_cell + rand * grid_size;

        rand_point = [min(rand_x, x_limites(2)), min(rand_y, y_limites(2))];
    else
        rand_point = [rand*(x_limites(2)-x_limites(1)), rand*(y_limites(2)-y_limites(1))];
    end
end
