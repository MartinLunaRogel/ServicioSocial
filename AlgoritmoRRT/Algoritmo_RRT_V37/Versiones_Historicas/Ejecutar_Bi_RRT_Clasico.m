function Ejecutar_Bi_RRT_Clasico()
    % Llamada a las variables del GUI
    global obstaculos punto_inicio punto_final ax;

    inicio = punto_inicio;
    objetivo = punto_final;

    % Parámetros escalados para el nuevo lienzo de 500x500
    x_limites = [0, 500];
    y_limites = [0, 500];
    tolerancia = 15;
    max_iter = 7000;
    step_size = 15;
    prob_objetivo = 0.10;
    grid_size = 25;
    prob_bias = 0.12;

    num_celdas_x = ceil((x_limites(2) - x_limites(1)) / grid_size);
    num_celdas_y = ceil((y_limites(2) - y_limites(1)) / grid_size);

    nodes_inicio = inicio;
    parent_inicio = 0;
    node_density_map_inicio = zeros(num_celdas_y, num_celdas_x);

    nodes_objetivo = objetivo;
    parent_objetivo = 0;
    node_density_map_objetivo = zeros(num_celdas_y, num_celdas_x);

    is_connected = false;
    idx_union_inicio = 0;
    idx_union_objetivo = 0;
    nodos_colisionados = 0;
    iteraciones_totales = 0;

    [col_idx_inicio, row_idx_inicio] = Mapeo_Densidad_Grid_V20(inicio, x_limites, y_limites, grid_size);
    node_density_map_inicio(row_idx_inicio, col_idx_inicio) = 1;

    [col_idx_objetivo, row_idx_objetivo] = Mapeo_Densidad_Grid_V20(objetivo, x_limites, y_limites, grid_size);
    node_density_map_objetivo(row_idx_objetivo, col_idx_objetivo) = 1;

    tic;

    for i = 1:max_iter
        iteraciones_totales = i;

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

        rand_point = Generar_Punto_Aleatorio_V20(other_nodes, current_density_map, x_limites, y_limites, grid_size, prob_objetivo, prob_bias);

        dif = current_nodes - rand_point;
        distancias = sqrt(sum(dif.^2, 2));
        [~, idx] = min(distancias);
        nearest_node = current_nodes(idx, :);

        direction = (rand_point - nearest_node);
        direction = direction / norm(direction);
        new_node = nearest_node + step_size * direction;

        if ~Verificar_Colision_Segmento_V20(nearest_node, new_node, obstaculos)
            current_nodes = [current_nodes; new_node];
            current_parent(end+1) = idx;
            new_node_idx = size(current_nodes, 1);

            [col_idx_new, row_idx_new] = Mapeo_Densidad_Grid_V20(new_node, x_limites, y_limites, grid_size);
            current_density_map(row_idx_new, col_idx_new) = current_density_map(row_idx_new, col_idx_new) + 1;

            plot(ax, [nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], current_tree_color);
            if mod(i, 50) == 0; drawnow; end

            dif_other = other_nodes - new_node;
            distancias_other = sqrt(sum(dif_other.^2, 2));
            [min_dist_to_other, idx_nearest_in_other] = min(distancias_other);
            nearest_other_node = other_nodes(idx_nearest_in_other, :);

            if min_dist_to_other < tolerancia
                if ~Verificar_Colision_Segmento_V20(new_node, nearest_other_node, obstaculos)
                    disp('Objetivo alcanzado! Conexión entre árboles encontrada.');
                    is_connected = true;

                    if tree_id == 1
                        idx_union_inicio = new_node_idx;
                        idx_union_objetivo = idx_nearest_in_other;
                    else
                        idx_union_objetivo = new_node_idx;
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

            if is_connected; break; end
        else
           nodos_colisionados = nodos_colisionados + 1;
        end
    end

    tiempo_busqueda_inicial = toc;

    if is_connected
        total_nodos_encontrados = size(nodes_inicio, 1) + size(nodes_objetivo, 1);

        [path_coords_inicio, path_indices_inicio] = Reconstruir_Camino_RRT_V20(idx_union_inicio, nodes_inicio, parent_inicio);
        [path_coords_objetivo, path_indices_objetivo] = Reconstruir_Camino_RRT_V20(idx_union_objetivo, nodes_objetivo, parent_objetivo);

        Limpiar_Dibujos_V20(ax, 'union');
        path_coords_inicial_completo = Dibujar_Ruta_Final_V20(ax, path_coords_inicio, path_coords_objetivo, 'inicial');

        disp('Iniciando optimización de caminos (RRT* post-procesamiento)...');
        tic;

        new_parent_inicio = Optimizar_Ruta_Final_V20(ax, nodes_inicio, parent_inicio, path_indices_inicio, obstaculos);
        new_parent_objetivo = Optimizar_Ruta_Final_V20(ax, nodes_objetivo, parent_objetivo, path_indices_objetivo, obstaculos);

        tiempo_optimizacion = toc;

        Limpiar_Dibujos_V20(ax, 'fase_optimizacion');

        [path_final_coords_inicio, ~] = Reconstruir_Camino_RRT_V20(idx_union_inicio, nodes_inicio, new_parent_inicio);
        [path_final_coords_objetivo, ~] = Reconstruir_Camino_RRT_V20(idx_union_objetivo, nodes_objetivo, new_parent_objetivo);

        path_final_coords_completo = Dibujar_Ruta_Final_V20(ax, path_final_coords_inicio, path_final_coords_objetivo, 'final');
        nodos_camino_final = size(path_final_coords_completo, 1);

        disp('Optimización finalizada. Camino más corto trazado.');

        Exportar_Reporte_Metricas_V20(path_final_coords_completo, tiempo_busqueda_inicial, tiempo_optimizacion, total_nodos_encontrados, nodos_camino_final, iteraciones_totales, nodos_colisionados);
    else
        disp('No se encontró conexión dentro del número máximo de iteraciones.');
    end
end
