% optimizar_camino_rrt_star.m
% =========================================================================
% === FUNCION DE OPTIMIZACIÓN DE CAMINO (Post-Procesamiento RRT*) ===
% =========================================================================

function new_parent = optimizar_camino_rrt_star(nodes, parent, path_indices, obstaculos, verificar_colision, distancia_muestreo)

    new_parent = parent;

    % Se define un handle a la figura actual para dibujar
    h_fig = gcf;

    disp('Iniciando visualización dinámica de la optimización...');

    % Recorremos todos los nodos del camino original, excepto el nodo de inicio (índice 1).
    for i = 2:length(path_indices)

        idx_hijo = path_indices(i);
        hijo = nodes(idx_hijo, :);

        % 1. Calcular el costo actual (Distancia de inicio a hijo)
        % === CAMBIO CLAVE: Llama a la funcion externa calcular_costo_camino ===
        current_cost_to_hijo = calcular_costo_camino(idx_hijo, nodes, new_parent);

        % Guardamos el padre actual antes de la búsqueda (para fines de dibujo)
        idx_padre_actual = new_parent(idx_hijo);

        % 2. Intentar optimizar el padre (Rewiring)
        for idx_padre_candidato = 1:size(nodes, 1)

            if idx_padre_candidato == idx_hijo || idx_padre_candidato == 1
                continue;
            end

            padre_candidato = nodes(idx_padre_candidato, :);
            dist_padre_a_hijo = norm(hijo - padre_candidato);

            % Calcular el costo total para llegar al padre candidato
            % === CAMBIO CLAVE: Llama a la funcion externa calcular_costo_camino ===
            costo_to_candidato = calcular_costo_camino(idx_padre_candidato, nodes, new_parent);
            potential_cost = costo_to_candidato + dist_padre_a_hijo;

            % === CONDICIÓN CLAVE DE OPTIMIZACIÓN ===
            if potential_cost < current_cost_to_hijo

                % Verificar la colisión para el nuevo enlace propuesto
                if ~verificar_colision(padre_candidato, hijo, obstaculos, distancia_muestreo)

                    % ¡OPTIMIZACIÓN ENCONTRADA!

                    % ===================================================
                    % === VISUALIZACIÓN DINÁMICA DEL REWIRING ===
                    % ===================================================

                    % 1. Dibujar el nuevo enlace (en un color temporal, e.g., amarillo)
                    plot([padre_candidato(1), hijo(1)], [padre_candidato(2), hijo(2)], 'y--', 'LineWidth', 1.5);

                    % 2. Actualizar el vector de padres
                    new_parent(idx_hijo) = idx_padre_candidato;

                    % 3. Actualizar el costo del hijo
                    current_cost_to_hijo = potential_cost;

                    % 4. Redibujar la figura para mostrar el cambio
                    drawnow;
                end
            end
        end
    end
end
