function Ejecutar_RRT_Voraz()
    global obstaculos punto_inicio punto_final ax;

    % Parámetros V10 adaptados al lienzo de 500x500
    x_limites = [0, 500]; y_limites = [0, 500];
    inicio = punto_inicio; objetivo = punto_final;
    tolerancia = 15; max_iter = 7000; step_size = 8;
    prob_objetivo = 0.05; grid_size = 25; prob_bias = 0.12;
    distancia_muestreo = 2.5;

    num_celdas_x = ceil((x_limites(2) - x_limites(1)) / grid_size);
    num_celdas_y = ceil((y_limites(2) - y_limites(1)) / grid_size);
    node_density_map = zeros(num_celdas_y, num_celdas_x);

    nodes = inicio; parent = 0;
    iteraciones_totales = 0; nodos_colisionados = 0;
    is_connected = false;

    tic;
    for i = 1:max_iter
        iteraciones_totales = i;

        % Muestreo (Bias de Objetivo + Densidad)
        r_prob = rand;
        if r_prob < prob_objetivo, rand_point = objetivo;
        elseif r_prob < prob_objetivo + prob_bias
            epsilon = 0.01;
            prob_matrix = 1 ./ (node_density_map.^2 + epsilon);
            prob_vector = prob_matrix(:) / sum(prob_matrix(:));
            selected_cell_idx = randsample(numel(prob_vector), 1, true, prob_vector);
            [row_idx, col_idx] = ind2sub(size(node_density_map), selected_cell_idx);
            rand_point = [(col_idx-1)*grid_size + rand*grid_size, (row_idx-1)*grid_size + rand*grid_size];
            rand_point = [min(rand_point(1), 500), min(rand_point(2), 500)];
        else, rand_point = [rand*500, rand*500];
        end

        % Voraz: Buscar nodo más cercano
        distancias = sqrt(sum((nodes - rand_point).^2, 2));
        [~, idx] = min(distancias);
        nearest_node = nodes(idx, :);

        direction = (rand_point - nearest_node);
        direction = direction / norm(direction);
        new_node = nearest_node + step_size * direction;

        if ~Verificar_Colision_Segmento_V10(nearest_node, new_node, obstaculos, distancia_muestreo)
            nodes = [nodes; new_node];
            parent(end+1) = idx;

            % Actualizar densidad
            c_idx = max(1, ceil(new_node(1)/grid_size)); r_idx = max(1, ceil(new_node(2)/grid_size));
            if r_idx <= num_celdas_y && c_idx <= num_celdas_x
                node_density_map(r_idx, c_idx) = node_density_map(r_idx, c_idx) + 1;
            end

            plot(ax, [nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], 'b');
            if mod(i, 50) == 0, drawnow; end

            if norm(new_node - objetivo) < tolerancia
                is_connected = true;
                break;
            end
        else
            nodos_colisionados = nodos_colisionados + 1;
        end
    end
    tiempo_busqueda = toc;

    if is_connected
        % Reconstruir ruta
        path = new_node; p = size(nodes, 1);
        while parent(p) ~= 0
            p = parent(p);
            path = [nodes(p,:); path];
        end
        plot(ax, path(:,1), path(:,2), 'r-', 'LineWidth', 2);
        plot(ax, path(:,1), path(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');

        Exportar_Reporte_Metricas_V10(path, tiempo_busqueda, 0, size(nodes,1), size(path,1), iteraciones_totales, nodos_colisionados);
    else
        disp('No se encontró conexión.');
    end
end
