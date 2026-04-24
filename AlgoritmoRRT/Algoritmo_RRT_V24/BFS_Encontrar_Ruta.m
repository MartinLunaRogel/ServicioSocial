% =========================================================================
% === FUNCION AUXILIAR: BÚSQUEDA DE RUTA EN LA RED DE ÁRBOLES (BFS) ===
% =========================================================================
function ruta = BFS_Encontrar_Ruta(matriz_adj, nodo_inicio, nodo_meta)
    num_nodos = size(matriz_adj, 1);
    visitado = false(1, num_nodos);
    padre = zeros(1, num_nodos);
    cola = nodo_inicio;
    visitado(nodo_inicio) = true;

    encontrado = false;
    while ~isempty(cola)
        actual = cola(1);
        cola(1) = []; % Extraer el primero de la cola

        if actual == nodo_meta
            encontrado = true;
            break;
        end

        for vecino = 1:num_nodos
            if matriz_adj(actual, vecino) == 1 && ~visitado(vecino)
                visitado(vecino) = true;
                padre(vecino) = actual;
                cola = [cola, vecino]; % Añadir al final de la cola
            end
        end
    end

    ruta = [];
    if encontrado
        % Reconstruir la ruta hacia atrás
        curr = nodo_meta;
        while curr ~= 0
            ruta = [curr, ruta];
            curr = padre(curr);
        end
    end
end
