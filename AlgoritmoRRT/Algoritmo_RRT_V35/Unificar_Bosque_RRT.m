% Unificar_Bosque_RRT.m
% =========================================================================
% === FUNCION PARA FUSIONAR EL BOSQUE EN UN SOLO GRAFO Y EXTRAER LA RUTA ===
% =========================================================================

function [nodes_global, parent_global, path_coords, path_indices] = Unificar_Bosque_RRT(bosque_nodos, bosque_padres, detalles_conexiones, num_arboles)

    % 1. Calcular cuántos nodos hay en total y sus posiciones (offsets)
    offsets = zeros(1, num_arboles);
    total_nodos = 0;
    for k = 1:num_arboles
        offsets(k) = total_nodos;
        total_nodos = total_nodos + size(bosque_nodos{k}, 1);
    end

    % 2. Meter todas las coordenadas en una sola lista gigante
    nodes_global = zeros(total_nodos, 2);
    for k = 1:num_arboles
        idx_start = offsets(k) + 1;
        idx_end = offsets(k) + size(bosque_nodos{k}, 1);
        nodes_global(idx_start:idx_end, :) = bosque_nodos{k};
    end

    % 3. Crear el mapa de parentesco (Adyacencia)
    adyacencia = cell(total_nodos, 1);
    for k = 1:num_arboles
        padres = bosque_padres{k};
        for i = 2:length(padres) % saltar la raiz que es 0
            hijo_global = offsets(k) + i;
            padre_global = offsets(k) + padres(i);
            adyacencia{hijo_global} = [adyacencia{hijo_global}, padre_global];
            adyacencia{padre_global} = [adyacencia{padre_global}, hijo_global];
        end
    end

    % 4. Conectar los puentes entre árboles distintos
    for i = 1:num_arboles
        for j = i+1:num_arboles
            if ~isempty(detalles_conexiones{i, j})
                nodo_en_i = offsets(i) + detalles_conexiones{i, j}(1);
                nodo_en_j = offsets(j) + detalles_conexiones{i, j}(2);
                adyacencia{nodo_en_i} = [adyacencia{nodo_en_i}, nodo_en_j];
                adyacencia{nodo_en_j} = [adyacencia{nodo_en_j}, nodo_en_i];
            end
        end
    end

    % 5. Trazar la línea directa usando un BFS sobre el Mega-Árbol
    idx_inicio_global = offsets(1) + 1;
    idx_objetivo_global = offsets(2) + 1; % El objetivo siempre fue el árbol 2

    parent_global = zeros(total_nodos, 1);
    visitado = false(total_nodos, 1);
    cola = idx_inicio_global;
    visitado(idx_inicio_global) = true;

    while ~isempty(cola)
        actual = cola(1);
        cola(1) = [];

        if actual == idx_objetivo_global
            break;
        end

        vecinos = adyacencia{actual};
        for v = vecinos
            if ~visitado(v)
                visitado(v) = true;
                parent_global(v) = actual;
                cola = [cola, v];
            end
        end
    end

    % 6. Reconstruir la ruta final
    path_indices = [];
    curr = idx_objetivo_global;
    while curr ~= 0
        path_indices = [curr; path_indices];
        curr = parent_global(curr);
    end
    path_coords = nodes_global(path_indices, :);
end
