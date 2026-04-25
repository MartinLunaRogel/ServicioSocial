function rand_point = Generar_Punto_Aleatorio(other_nodes, current_density_map, x_limites, y_limites, grid_size, prob_objetivo, prob_bias, semillas, mapa_discreto)
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

    % 3. NUEVO: Muestreo blindado por Grid Caching
    else
        punto_valido = false;

        while ~punto_valido
            idx_semilla = randi(size(semillas, 1));
            semilla_elegida = semillas(idx_semilla, :);

            ruido_x = (rand - 0.5) * 30;
            ruido_y = (rand - 0.5) * 30;

            rand_x = semilla_elegida(1) + ruido_x;
            rand_y = semilla_elegida(2) + ruido_y;

            rand_x = max(x_limites(1), min(rand_x, x_limites(2)));
            rand_y = max(y_limites(1), min(rand_y, y_limites(2)));

            % Consultamos la matriz de ocupación.
            % MATLAB empieza sus índices en 1, por lo que sumamos 1 al redondeo.
            % Al ser matriz (fila, columna), consultamos (Y, X).
            idx_x = round(rand_x) + 1;
            idx_y = round(rand_y) + 1;

            % Si la coordenada cae en un 0 (calle libre), aceptamos el punto
            if mapa_discreto(idx_y, idx_x) == 0
                punto_valido = true;
                rand_point = [rand_x, rand_y];
            end
            % Si cae en 1 (edificio), el ciclo se repite en milisegundos
        end
    end
end
