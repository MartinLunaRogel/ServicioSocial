% ==============================================================================
% === FUNCION AUXILIAR: BÚSQUEDA DE RUTA EN LA RED DE ÁRBOLES (BFS) ===

% Esta funcion actua como un rastreador. Cunando dos arboles ya se conectaron,
% esta funcion navega por las ramas conectadas para encontrar la ruta exacta
% desde el punto de inicio hasta el punto objetivo.
% Utiliza un algoritmo clasico llamado BFS ((Busqueda den anchura)
% ==============================================================================

function ruta = BFS_Encontrar_Ruta(matriz_adj, nodo_inicio, nodo_meta)
    num_nodos = size(matriz_adj, 1);  % saber cuantos nodos hay en la matriz
    visitado = false(1, num_nodos);   % se crea lista booleana para ver que nodos ya se visitaron
    padre = zeros(1, num_nodos);      % se crea una lista de ceros para ver que nodo es padre de cual
    cola = nodo_inicio;               % se crea una cola para poner el nodo de partida como el primer en turno
    visitado(nodo_inicio) = true;     % marcar el nodo de partida como ya "visitado" para no volver a el

    encontrado = false;               % Interruptore de salida, se enciende cuando ya se encontro el primer camino al nodo final

    % --- CICLO DE EXPLORACION (mIENTRAS HAYA NODOS EN LA FILA) ---
    while ~isempty(cola)
        actual = cola(1);             % Se toma el primer dnodo en la fina de espara
        cola(1) = [];                 % Se saca de la fila de espera

        if actual == nodo_meta        % Si el nodo actual revidando es el nodo final
            encontrado = true;        % Se enciende el interruptor de salida
            break;
        end

        for vecino = 1:num_nodos        % si no es la meta se revisa todos los nodos "vecinos" de este nodo
            if matriz_adj(actual, vecino) == 1 && ~visitado(vecino) % Se verifica si estan conectados y si sun no ha sido visitado
                visitado(vecino) = true;% Se marca como visitado
                padre(vecino) = actual; % Se anota que se llego a este nodo graciqs al nodo actual (su nodo padre)
                cola = [cola, vecino];  % Añadir al final de la cola
            end
        end
    end

    % --- RECONSTRUCCIÓN DEL CAMINO (DE ATRÁS HACIA ADELANTE) ---

    ruta = [];                        % Lista vacia donde se guardara solo el camino final
    if encontrado                     % Entra solo si el interruptor de encontraso esta encendido
        curr = nodo_meta;             % Se empieza a caminar hacia atras en los nodos iniciando desde el nodo objetivo
        while curr ~= 0               % Mientras no se llegue al "vacio", el inicio no tiene padre osea es 0
            ruta = [curr, ruta];      % Se agrega el nodo actual al principio de la lista de ruta
            curr = padre(curr);       % Se retrocede un paso, preguntando quien es el padre del nodo actual
        end
    end
end
