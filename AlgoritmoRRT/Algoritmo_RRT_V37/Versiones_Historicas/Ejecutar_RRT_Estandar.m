function Ejecutar_RRT_Estandar()
    % Llamada a las variables del GUI
    global obstaculos punto_inicio punto_final ax;

    % =========================================================================
    %  === PARAMETROS DEL ESPACIO Y DEL ALGORITMO (Escalados a 500x500) ===
    % =========================================================================
    x_limites = [0, 500];
    y_limites = [0, 500];
    inicio = punto_inicio;
    objetivo = punto_final;
    tolerancia = 15;
    max_iter = 7000;
    step_size = 15;
    prob_objetivo = 0.05;
    grid_size = 25;
    prob_bias = 0.12;

    num_celdas_x = ceil((x_limites(2) - x_limites(1)) / grid_size);
    num_celdas_y = ceil((y_limites(2) - y_limites(1)) / grid_size);

    distancia_muestreo = 2.5;

    % =========================================================================
    %  === INICIALIZAR ESTRUCTURAS DOBLES ===
    % =========================================================================
    nodes_inicio = inicio;
    parent_inicio = 0;
    node_density_map_inicio = zeros(num_celdas_y, num_celdas_x);

    nodes_objetivo = objetivo;
    parent_objetivo = 0;
    node_density_map_objetivo = zeros(num_celdas_y, num_celdas_x);

    is_connected = false;
    idx_union_inicio = 0;
    idx_union_objetivo = 0;

    % Contadores agregados para las métricas
    iteraciones_totales = 0;
    nodos_colisionados = 0;

    % Inicialización del mapa de densidad
    [col_idx_inicio, row_idx_inicio] = Mapeo_Densidad_Grid_V15(inicio, x_limites, y_limites, grid_size);
    node_density_map_inicio(row_idx_inicio, col_idx_inicio) = node_density_map_inicio(row_idx_inicio, col_idx_inicio) + 1;

    [col_idx_objetivo, row_idx_objetivo] = Mapeo_Densidad_Grid_V15(objetivo, x_limites, y_limites, grid_size);
    node_density_map_objetivo(row_idx_objetivo, col_idx_objetivo) = node_density_map_objetivo(row_idx_objetivo, col_idx_objetivo) + 1;

    tic;

    for i = 1:max_iter
        iteraciones_totales = i; % Registro de la iteración actual

        if mod(i, 2) == 1
            current_nodes = nodes_inicio;
            current_parent = parent_inicio;
            current_density_map = node_density_map_inicio;
            current_tree_color = 'b';
            other_nodes = nodes_objetivo;
            tree_id = 1;
        else
            current_nodes = nodes_objetivo;
            current_parent = parent_objetivo;
            current_density_map = node_density_map_objetivo;
            current_tree_color = 'c';
            other_nodes = nodes_inicio;
            tree_id = 2;
        end

        % Generar un punto aleatorio en el espacio (Muestreo con Bias INLINE)
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
            rand_x = min(x_min_cell + rand * grid_size, x_limites(2));
            rand_y = min(y_min_cell + rand * grid_size, y_limites(2));
            rand_point = [rand_x, rand_y];
        else
            rand_point = [rand*(x_limites(2)-x_limites(1)), rand*(y_limites(2)-y_limites(1))];
        end

        dif = current_nodes - rand_point;
        distancias = sqrt(sum(dif.^2, 2));
        [~, idx] = min(distancias);
        nearest_node = current_nodes(idx, :);

        direction = (rand_point - nearest_node);
        direction = direction / norm(direction);
        new_node = nearest_node + step_size * direction;

        if ~Verificar_Colision_Segmento_V15(nearest_node, new_node, obstaculos, distancia_muestreo)
            current_nodes = [current_nodes; new_node];
            current_parent(end+1) = idx;
            new_node_idx_in_current_tree = size(current_nodes, 1);

            [col_idx_new_node, row_idx_new_node] = Mapeo_Densidad_Grid_V15(new_node, x_limites, y_limites, grid_size);
            current_density_map(row_idx_new_node, col_idx_new_node) = current_density_map(row_idx_new_node, col_idx_new_node) + 1;

            plot(ax, [nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], current_tree_color);
            if mod(i, 50) == 0; drawnow; end

            dif_other = other_nodes - new_node;
            distancias_other = sqrt(sum(dif_other.^2, 2));
            [min_dist_to_other, idx_nearest_in_other] = min(distancias_other);
            nearest_other_node = other_nodes(idx_nearest_in_other, :);

            if min_dist_to_other < tolerancia
                if ~Verificar_Colision_Segmento_V15(new_node, nearest_other_node, obstaculos, distancia_muestreo)
                    disp('Objetivo alcanzado! Conexión entre árboles encontrada.');
                    is_connected = true;

                    if tree_id == 1
                        idx_union_inicio = new_node_idx_in_current_tree;
                        idx_union_objetivo = idx_nearest_in_other;
                    else
                        idx_union_objetivo = new_node_idx_in_current_tree;
                        idx_union_inicio = idx_nearest_in_other;
                    end

                    plot(ax, [new_node(1), nearest_other_node(1)], [new_node(2), nearest_other_node(2)], 'm--', 'LineWidth', 1.5);
                    drawnow;
                end
            end

            if tree_id == 1
                nodes_inicio = current_nodes;
                parent_inicio = current_parent;
                node_density_map_inicio = current_density_map;
            else
                nodes_objetivo = current_nodes;
                parent_objetivo = current_parent;
                node_density_map_objetivo = current_density_map;
            end

            if is_connected
                break;
            end
        else
            nodos_colisionados = nodos_colisionados + 1; % Registro de impacto
        end
    end

    tiempo_busqueda_inicial = toc;

    if is_connected
        total_nodos_encontrados = size(nodes_inicio, 1) + size(nodes_objetivo, 1);

        % RECONSTRUCCIÓN INLINE DE LA V15
        path_indices_inicio = [];
        p_inicio = idx_union_inicio;
        while parent_inicio(p_inicio) ~= 0
            path_indices_inicio = [p_inicio; path_indices_inicio];
            p_inicio = parent_inicio(p_inicio);
        end
        path_indices_inicio = [1; path_indices_inicio];
        path_coords_inicio = nodes_inicio(path_indices_inicio, :);

        path_indices_objetivo = [];
        p_objetivo = idx_union_objetivo;
        while parent_objetivo(p_objetivo) ~= 0
            path_indices_objetivo = [p_objetivo; path_indices_objetivo];
            p_objetivo = parent_objetivo(p_objetivo);
        end
        path_indices_objetivo = [1; path_indices_objetivo];
        path_coords_objetivo = nodes_objetivo(path_indices_objetivo, :);

        path_coords_objetivo_invertido = flipud(path_coords_objetivo(2:end, :));
        path_coords_inicial_completo = [path_coords_inicio; path_coords_objetivo_invertido];

        h_union_line = findobj(ax, 'Type', 'line', 'Color', 'm', 'LineWidth', 1.5);
        delete(h_union_line);

        plot(ax, path_coords_inicial_completo(:,1), path_coords_inicial_completo(:,2), 'r-', 'LineWidth', 2);
        plot(ax, path_coords_inicial_completo(:,1), path_coords_inicial_completo(:,2), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'black');

        disp('Iniciando optimización de caminos (RRT* post-procesamiento)...');
        tic;

        disp('Optimizando T_inicio...');
        new_parent_inicio = Optimizar_Ruta_Final_V15(ax, nodes_inicio, parent_inicio, path_indices_inicio, obstaculos, @Verificar_Colision_Segmento_V15, distancia_muestreo);

        disp('Optimizando T_objetivo...');
        new_parent_objetivo = Optimizar_Ruta_Final_V15(ax, nodes_objetivo, parent_objetivo, path_indices_objetivo, obstaculos, @Verificar_Colision_Segmento_V15, distancia_muestreo);

        tiempo_optimizacion = toc;

        h_lines = findobj(ax, 'Type', 'line', 'LineWidth', 2);
        delete(h_lines);
        h_temp_lines = findobj(ax, 'Type', 'line', 'Color', 'g', 'LineWidth', 1.5);
        delete(h_temp_lines);
        h_markers = findobj(ax, 'Type', 'line', 'MarkerSize', 4);
        delete(h_markers);

        final_path_indices_inicio = [];
        p_final_inicio = idx_union_inicio;
        while new_parent_inicio(p_final_inicio) ~= 0
            final_path_indices_inicio = [p_final_inicio; final_path_indices_inicio];
            p_final_inicio = new_parent_inicio(p_final_inicio);
        end
        final_path_indices_inicio = [1; final_path_indices_inicio];
        path_final_coords_inicio = nodes_inicio(final_path_indices_inicio, :);

        final_path_indices_objetivo = [];
        p_final_objetivo = idx_union_objetivo;
        while new_parent_objetivo(p_final_objetivo) ~= 0
            final_path_indices_objetivo = [p_final_objetivo; final_path_indices_objetivo];
            p_final_objetivo = new_parent_objetivo(p_final_objetivo);
        end
        final_path_indices_objetivo = [1; final_path_indices_objetivo];
        path_final_coords_objetivo = nodes_objetivo(final_path_indices_objetivo, :);

        path_final_coords_objetivo_invertido = flipud(path_final_coords_objetivo(2:end, :));
        path_final_coords_completo = [path_final_coords_inicio; path_final_coords_objetivo_invertido];
        nodos_camino_final = size(path_final_coords_completo, 1);

        plot(ax, path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'r-', 'LineWidth', 2);
        plot(ax, path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');

        disp('Optimización finalizada. Camino más corto trazado.');

        % EXPORTACIÓN DE MÉTRICAS AL ARCHIVO .TXT
        Exportar_Reporte_Metricas_V15(path_final_coords_completo, tiempo_busqueda_inicial, tiempo_optimizacion, total_nodos_encontrados, nodos_camino_final, iteraciones_totales, nodos_colisionados);

    else
        disp('No se encontró conexión dentro del número máximo de iteraciones.');
    end
end
