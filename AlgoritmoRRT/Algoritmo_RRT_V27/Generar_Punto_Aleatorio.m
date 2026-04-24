function rand_point = Generar_Punto_Aleatorio(other_nodes, current_density_map, x_limites, y_limites, grid_size, prob_objetivo, prob_bias, semillas)
    r_prob = rand;

    % 1. Muestrear hacia el otro árbol (Bias de Objetivo)
    if r_prob < prob_objetivo
        rand_point = other_nodes(1, :);

    % 2. Muestreo hacia zonas de baja densidad (Bias de Exploración)
    elseif r_prob < prob_objetivo + prob_bias
        epsilon = 0.01;
        prob_matrix = 1 ./ (current_density_map.^2 + epsilon);
        prob_vector = prob_matrix(:);
        prob_vector = prob_vector / sum(prob_vector);
        selected_cell_idx = randsample(numel(prob_vector), 1, true, prob_vector);
        [row_idx, col_idx] = ind2sub(size(current_density_map), selected_cell_idx);

        x_min_cell = (col_idx - 1) * grid_size + x_limites(1);
        y_min_cell = (row_idx - 1) * grid_size + y_limites(1);
        rand_point = [x_min_cell + rand * grid_size, y_min_cell + rand * grid_size];
        rand_point = [min(rand_point(1), x_limites(2)), min(rand_point(2), y_limites(2))];

    % 3. NUEVO: Muestreo basado en Calles (Semillas)
    else
        % Elegimos una semilla al azar (que ya sabemos que está en una calle libre)
        idx_semilla = randi(size(semillas, 1));
        semilla_elegida = semillas(idx_semilla, :);

        % Le sumamos un pequeño "ruido" aleatorio (ej. +/- 15 unidades)
        % para explorar los alrededores de esa calle, no solo el punto central.
        ruido_x = (rand - 0.5) * 30;
        ruido_y = (rand - 0.5) * 30;

        rand_x = semilla_elegida(1) + ruido_x;
        rand_y = semilla_elegida(2) + ruido_y;

        % Asegurar que no se salga de los bordes del mapa
        rand_point = [max(x_limites(1), min(rand_x, x_limites(2))), ...
                      max(y_limites(1), min(rand_y, y_limites(2)))];
    end
end
