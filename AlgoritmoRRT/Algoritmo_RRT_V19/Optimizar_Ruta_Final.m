% Optimizar_Ruta_Final
% =========================================================================
% === FUNCION DE OPTIMIZACIÓN DE CAMINO (Post-Procesamiento RRT*) ===
%
% Descripción: Esta función toma un árbol (nodes, parent) y los índices de
% una ruta inicial encontrada (path_indices) y los re-conecta (rewiring)
% para encontrar un camino de costo total menor.
% =========================================================================

function new_parent = Optimizar_Ruta_Final(nodes, parent, path_indices, obstaculos, Verificar_Colision_Segmento, distancia_muestreo)

    new_parent = parent;

    % Se define un handle a la figura actual para dibujar
    h_fig = gcf;

    % Recorremos todos los nodos del camino original, excepto el nodo de inicio (índice 1).
    for i = 2:length(path_indices)

        idx_hijo = path_indices(i);
        hijo = nodes(idx_hijo, :);

        % 1. Calcular el costo actual (Distancia de inicio a hijo)
        % === Llama a la funcion externa Calcular_Costo_De_Ruta ===
        current_cost_to_hijo = Calcular_Costo_De_Ruta(idx_hijo, nodes, new_parent);

        % Guardamos el padre actual antes de la búsqueda (para fines de dibujo)
        % idx_padre_actual = new_parent(idx_hijo); % (Línea no usada, mantenida como comentario)

        % 2. Intentar optimizar el padre (Rewiring): Revisa TODOS los nodos del árbol como posibles padres.
        for idx_padre_candidato = 1:size(nodes, 1)

            if idx_padre_candidato == idx_hijo || idx_padre_candidato == 1
                continue; % No re-conectar a sí mismo o al nodo raíz (índice 1)
            end

            padre_candidato = nodes(idx_padre_candidato, :);
            dist_padre_a_hijo = norm(hijo - padre_candidato); % Distancia del nuevo segmento

            % Calcular el costo total para llegar al padre candidato
            % === Llama a la funcion externa Calcular_Costo_De_Ruta ===
            costo_to_candidato = Calcular_Costo_De_Ruta(idx_padre_candidato, nodes, new_parent);
            potential_cost = costo_to_candidato + dist_padre_a_hijo; % Costo total propuesto

            % === CONDICIÓN CLAVE DE OPTIMIZACIÓN ===
            if potential_cost < current_cost_to_hijo

                % Verificar la colisión para el nuevo enlace propuesto
                if ~Verificar_Colision_Segmento(padre_candidato, hijo, obstaculos, distancia_muestreo)

                    % ¡OPTIMIZACIÓN ENCONTRADA! (Rewiring)

                    % ===================================================
                    % === VISUALIZACIÓN DINÁMICA DEL REWIRING ===
                    % ===================================================

                    % 1. Dibujar el nuevo enlace (en un color temporal, e.g., verde punteado)
                    plot([padre_candidato(1), hijo(1)], [padre_candidato(2), hijo(2)], 'g--', 'LineWidth', 1.5);

                    % 2. Actualizar el vector de padres
                    new_parent(idx_hijo) = idx_padre_candidato;

                    % 3. Actualizar el costo del hijo (para futuros rewiring dentro del mismo bucle)
                    current_cost_to_hijo = potential_cost;

                    % 4. Redibujar la figura para mostrar el cambio
                    drawnow;
                end
            end
        end
    end
end
